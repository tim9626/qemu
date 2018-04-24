/*
 *  Emulation of Linux signals
 *
 *  Copyright (c) 2003 Fabrice Bellard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include <sys/ucontext.h>
#include <sys/resource.h>

#include "qemu.h"
#include "qemu-common.h"
#include "target_signal.h"
#include "trace.h"
#include "signal-common.h"

struct target_sigaltstack target_sigaltstack_used = {
    .ss_sp = 0,
    .ss_size = 0,
    .ss_flags = TARGET_SS_DISABLE,
};

static struct target_sigaction sigact_table[TARGET_NSIG];

static void host_signal_handler(int host_signum, siginfo_t *info,
                                void *puc);

static uint8_t host_to_target_signal_table[_NSIG] = {
    [SIGHUP] = TARGET_SIGHUP,
    [SIGINT] = TARGET_SIGINT,
    [SIGQUIT] = TARGET_SIGQUIT,
    [SIGILL] = TARGET_SIGILL,
    [SIGTRAP] = TARGET_SIGTRAP,
    [SIGABRT] = TARGET_SIGABRT,
/*    [SIGIOT] = TARGET_SIGIOT,*/
    [SIGBUS] = TARGET_SIGBUS,
    [SIGFPE] = TARGET_SIGFPE,
    [SIGKILL] = TARGET_SIGKILL,
    [SIGUSR1] = TARGET_SIGUSR1,
    [SIGSEGV] = TARGET_SIGSEGV,
    [SIGUSR2] = TARGET_SIGUSR2,
    [SIGPIPE] = TARGET_SIGPIPE,
    [SIGALRM] = TARGET_SIGALRM,
    [SIGTERM] = TARGET_SIGTERM,
#ifdef SIGSTKFLT
    [SIGSTKFLT] = TARGET_SIGSTKFLT,
#endif
    [SIGCHLD] = TARGET_SIGCHLD,
    [SIGCONT] = TARGET_SIGCONT,
    [SIGSTOP] = TARGET_SIGSTOP,
    [SIGTSTP] = TARGET_SIGTSTP,
    [SIGTTIN] = TARGET_SIGTTIN,
    [SIGTTOU] = TARGET_SIGTTOU,
    [SIGURG] = TARGET_SIGURG,
    [SIGXCPU] = TARGET_SIGXCPU,
    [SIGXFSZ] = TARGET_SIGXFSZ,
    [SIGVTALRM] = TARGET_SIGVTALRM,
    [SIGPROF] = TARGET_SIGPROF,
    [SIGWINCH] = TARGET_SIGWINCH,
    [SIGIO] = TARGET_SIGIO,
    [SIGPWR] = TARGET_SIGPWR,
    [SIGSYS] = TARGET_SIGSYS,
    /* next signals stay the same */
    /* Nasty hack: Reverse SIGRTMIN and SIGRTMAX to avoid overlap with
       host libpthread signals.  This assumes no one actually uses SIGRTMAX :-/
       To fix this properly we need to do manual signal delivery multiplexed
       over a single host signal.  */
    [__SIGRTMIN] = __SIGRTMAX,
    [__SIGRTMAX] = __SIGRTMIN,
};
static uint8_t target_to_host_signal_table[_NSIG];

int host_to_target_signal(int sig)
{
    if (sig < 0 || sig >= _NSIG)
        return sig;
    return host_to_target_signal_table[sig];
}

int target_to_host_signal(int sig)
{
    if (sig < 0 || sig >= _NSIG)
        return sig;
    return target_to_host_signal_table[sig];
}

static inline void target_sigaddset(target_sigset_t *set, int signum)
{
    signum--;
    abi_ulong mask = (abi_ulong)1 << (signum % TARGET_NSIG_BPW);
    set->sig[signum / TARGET_NSIG_BPW] |= mask;
}

static inline int target_sigismember(const target_sigset_t *set, int signum)
{
    signum--;
    abi_ulong mask = (abi_ulong)1 << (signum % TARGET_NSIG_BPW);
    return ((set->sig[signum / TARGET_NSIG_BPW] & mask) != 0);
}

void host_to_target_sigset_internal(target_sigset_t *d,
                                    const sigset_t *s)
{
    int i;
    target_sigemptyset(d);
    for (i = 1; i <= TARGET_NSIG; i++) {
        if (sigismember(s, i)) {
            target_sigaddset(d, host_to_target_signal(i));
        }
    }
}

void host_to_target_sigset(target_sigset_t *d, const sigset_t *s)
{
    target_sigset_t d1;
    int i;

    host_to_target_sigset_internal(&d1, s);
    for(i = 0;i < TARGET_NSIG_WORDS; i++)
        d->sig[i] = tswapal(d1.sig[i]);
}

void target_to_host_sigset_internal(sigset_t *d,
                                    const target_sigset_t *s)
{
    int i;
    sigemptyset(d);
    for (i = 1; i <= TARGET_NSIG; i++) {
        if (target_sigismember(s, i)) {
            sigaddset(d, target_to_host_signal(i));
        }
    }
}

void target_to_host_sigset(sigset_t *d, const target_sigset_t *s)
{
    target_sigset_t s1;
    int i;

    for(i = 0;i < TARGET_NSIG_WORDS; i++)
        s1.sig[i] = tswapal(s->sig[i]);
    target_to_host_sigset_internal(d, &s1);
}

void host_to_target_old_sigset(abi_ulong *old_sigset,
                               const sigset_t *sigset)
{
    target_sigset_t d;
    host_to_target_sigset(&d, sigset);
    *old_sigset = d.sig[0];
}

void target_to_host_old_sigset(sigset_t *sigset,
                               const abi_ulong *old_sigset)
{
    target_sigset_t d;
    int i;

    d.sig[0] = *old_sigset;
    for(i = 1;i < TARGET_NSIG_WORDS; i++)
        d.sig[i] = 0;
    target_to_host_sigset(sigset, &d);
}

int block_signals(void)
{
    TaskState *ts = (TaskState *)thread_cpu->opaque;
    sigset_t set;

    /* It's OK to block everything including SIGSEGV, because we won't
     * run any further guest code before unblocking signals in
     * process_pending_signals().
     */
    sigfillset(&set);
    sigprocmask(SIG_SETMASK, &set, 0);

    return atomic_xchg(&ts->signal_pending, 1);
}

/* Wrapper for sigprocmask function
 * Emulates a sigprocmask in a safe way for the guest. Note that set and oldset
 * are host signal set, not guest ones. Returns -TARGET_ERESTARTSYS if
 * a signal was already pending and the syscall must be restarted, or
 * 0 on success.
 * If set is NULL, this is guaranteed not to fail.
 */
int do_sigprocmask(int how, const sigset_t *set, sigset_t *oldset)
{
    TaskState *ts = (TaskState *)thread_cpu->opaque;

    if (oldset) {
        *oldset = ts->signal_mask;
    }

    if (set) {
        int i;

        if (block_signals()) {
            return -TARGET_ERESTARTSYS;
        }

        switch (how) {
        case SIG_BLOCK:
            sigorset(&ts->signal_mask, &ts->signal_mask, set);
            break;
        case SIG_UNBLOCK:
            for (i = 1; i <= NSIG; ++i) {
                if (sigismember(set, i)) {
                    sigdelset(&ts->signal_mask, i);
                }
            }
            break;
        case SIG_SETMASK:
            ts->signal_mask = *set;
            break;
        default:
            g_assert_not_reached();
        }

        /* Silently ignore attempts to change blocking status of KILL or STOP */
        sigdelset(&ts->signal_mask, SIGKILL);
        sigdelset(&ts->signal_mask, SIGSTOP);
    }
    return 0;
}

#if !defined(TARGET_OPENRISC) && !defined(TARGET_NIOS2)
/* Just set the guest's signal mask to the specified value; the
 * caller is assumed to have called block_signals() already.
 */
void set_sigmask(const sigset_t *set)
{
    TaskState *ts = (TaskState *)thread_cpu->opaque;

    ts->signal_mask = *set;
}
#endif

/* siginfo conversion */

static inline void host_to_target_siginfo_noswap(target_siginfo_t *tinfo,
                                                 const siginfo_t *info)
{
    int sig = host_to_target_signal(info->si_signo);
    int si_code = info->si_code;
    int si_type;
    tinfo->si_signo = sig;
    tinfo->si_errno = 0;
    tinfo->si_code = info->si_code;

    /* This memset serves two purposes:
     * (1) ensure we don't leak random junk to the guest later
     * (2) placate false positives from gcc about fields
     *     being used uninitialized if it chooses to inline both this
     *     function and tswap_siginfo() into host_to_target_siginfo().
     */
    memset(tinfo->_sifields._pad, 0, sizeof(tinfo->_sifields._pad));

    /* This is awkward, because we have to use a combination of
     * the si_code and si_signo to figure out which of the union's
     * members are valid. (Within the host kernel it is always possible
     * to tell, but the kernel carefully avoids giving userspace the
     * high 16 bits of si_code, so we don't have the information to
     * do this the easy way...) We therefore make our best guess,
     * bearing in mind that a guest can spoof most of the si_codes
     * via rt_sigqueueinfo() if it likes.
     *
     * Once we have made our guess, we record it in the top 16 bits of
     * the si_code, so that tswap_siginfo() later can use it.
     * tswap_siginfo() will strip these top bits out before writing
     * si_code to the guest (sign-extending the lower bits).
     */

    switch (si_code) {
    case SI_USER:
    case SI_TKILL:
    case SI_KERNEL:
        /* Sent via kill(), tkill() or tgkill(), or direct from the kernel.
         * These are the only unspoofable si_code values.
         */
        tinfo->_sifields._kill._pid = info->si_pid;
        tinfo->_sifields._kill._uid = info->si_uid;
        si_type = QEMU_SI_KILL;
        break;
    default:
        /* Everything else is spoofable. Make best guess based on signal */
        switch (sig) {
        case TARGET_SIGCHLD:
            tinfo->_sifields._sigchld._pid = info->si_pid;
            tinfo->_sifields._sigchld._uid = info->si_uid;
            tinfo->_sifields._sigchld._status
                = host_to_target_waitstatus(info->si_status);
            tinfo->_sifields._sigchld._utime = info->si_utime;
            tinfo->_sifields._sigchld._stime = info->si_stime;
            si_type = QEMU_SI_CHLD;
            break;
        case TARGET_SIGIO:
            tinfo->_sifields._sigpoll._band = info->si_band;
            tinfo->_sifields._sigpoll._fd = info->si_fd;
            si_type = QEMU_SI_POLL;
            break;
        default:
            /* Assume a sigqueue()/mq_notify()/rt_sigqueueinfo() source. */
            tinfo->_sifields._rt._pid = info->si_pid;
            tinfo->_sifields._rt._uid = info->si_uid;
            /* XXX: potential problem if 64 bit */
            tinfo->_sifields._rt._sigval.sival_ptr
                = (abi_ulong)(unsigned long)info->si_value.sival_ptr;
            si_type = QEMU_SI_RT;
            break;
        }
        break;
    }

    tinfo->si_code = deposit32(si_code, 16, 16, si_type);
}

void tswap_siginfo(target_siginfo_t *tinfo,
                   const target_siginfo_t *info)
{
    int si_type = extract32(info->si_code, 16, 16);
    int si_code = sextract32(info->si_code, 0, 16);

    __put_user(info->si_signo, &tinfo->si_signo);
    __put_user(info->si_errno, &tinfo->si_errno);
    __put_user(si_code, &tinfo->si_code);

    /* We can use our internal marker of which fields in the structure
     * are valid, rather than duplicating the guesswork of
     * host_to_target_siginfo_noswap() here.
     */
    switch (si_type) {
    case QEMU_SI_KILL:
        __put_user(info->_sifields._kill._pid, &tinfo->_sifields._kill._pid);
        __put_user(info->_sifields._kill._uid, &tinfo->_sifields._kill._uid);
        break;
    case QEMU_SI_TIMER:
        __put_user(info->_sifields._timer._timer1,
                   &tinfo->_sifields._timer._timer1);
        __put_user(info->_sifields._timer._timer2,
                   &tinfo->_sifields._timer._timer2);
        break;
    case QEMU_SI_POLL:
        __put_user(info->_sifields._sigpoll._band,
                   &tinfo->_sifields._sigpoll._band);
        __put_user(info->_sifields._sigpoll._fd,
                   &tinfo->_sifields._sigpoll._fd);
        break;
    case QEMU_SI_FAULT:
        __put_user(info->_sifields._sigfault._addr,
                   &tinfo->_sifields._sigfault._addr);
        break;
    case QEMU_SI_CHLD:
        __put_user(info->_sifields._sigchld._pid,
                   &tinfo->_sifields._sigchld._pid);
        __put_user(info->_sifields._sigchld._uid,
                   &tinfo->_sifields._sigchld._uid);
        __put_user(info->_sifields._sigchld._status,
                   &tinfo->_sifields._sigchld._status);
        __put_user(info->_sifields._sigchld._utime,
                   &tinfo->_sifields._sigchld._utime);
        __put_user(info->_sifields._sigchld._stime,
                   &tinfo->_sifields._sigchld._stime);
        break;
    case QEMU_SI_RT:
        __put_user(info->_sifields._rt._pid, &tinfo->_sifields._rt._pid);
        __put_user(info->_sifields._rt._uid, &tinfo->_sifields._rt._uid);
        __put_user(info->_sifields._rt._sigval.sival_ptr,
                   &tinfo->_sifields._rt._sigval.sival_ptr);
        break;
    default:
        g_assert_not_reached();
    }
}

void host_to_target_siginfo(target_siginfo_t *tinfo, const siginfo_t *info)
{
    target_siginfo_t tgt_tmp;
    host_to_target_siginfo_noswap(&tgt_tmp, info);
    tswap_siginfo(tinfo, &tgt_tmp);
}

/* XXX: we support only POSIX RT signals are used. */
/* XXX: find a solution for 64 bit (additional malloced data is needed) */
void target_to_host_siginfo(siginfo_t *info, const target_siginfo_t *tinfo)
{
    /* This conversion is used only for the rt_sigqueueinfo syscall,
     * and so we know that the _rt fields are the valid ones.
     */
    abi_ulong sival_ptr;

    __get_user(info->si_signo, &tinfo->si_signo);
    __get_user(info->si_errno, &tinfo->si_errno);
    __get_user(info->si_code, &tinfo->si_code);
    __get_user(info->si_pid, &tinfo->_sifields._rt._pid);
    __get_user(info->si_uid, &tinfo->_sifields._rt._uid);
    __get_user(sival_ptr, &tinfo->_sifields._rt._sigval.sival_ptr);
    info->si_value.sival_ptr = (void *)(long)sival_ptr;
}

static int fatal_signal (int sig)
{
    switch (sig) {
    case TARGET_SIGCHLD:
    case TARGET_SIGURG:
    case TARGET_SIGWINCH:
        /* Ignored by default.  */
        return 0;
    case TARGET_SIGCONT:
    case TARGET_SIGSTOP:
    case TARGET_SIGTSTP:
    case TARGET_SIGTTIN:
    case TARGET_SIGTTOU:
        /* Job control signals.  */
        return 0;
    default:
        return 1;
    }
}

/* returns 1 if given signal should dump core if not handled */
static int core_dump_signal(int sig)
{
    switch (sig) {
    case TARGET_SIGABRT:
    case TARGET_SIGFPE:
    case TARGET_SIGILL:
    case TARGET_SIGQUIT:
    case TARGET_SIGSEGV:
    case TARGET_SIGTRAP:
    case TARGET_SIGBUS:
        return (1);
    default:
        return (0);
    }
}

void signal_init(void)
{
    TaskState *ts = (TaskState *)thread_cpu->opaque;
    struct sigaction act;
    struct sigaction oact;
    int i, j;
    int host_sig;

    /* generate signal conversion tables */
    for(i = 1; i < _NSIG; i++) {
        if (host_to_target_signal_table[i] == 0)
            host_to_target_signal_table[i] = i;
    }
    for(i = 1; i < _NSIG; i++) {
        j = host_to_target_signal_table[i];
        target_to_host_signal_table[j] = i;
    }

    /* Set the signal mask from the host mask. */
    sigprocmask(0, 0, &ts->signal_mask);

    /* set all host signal handlers. ALL signals are blocked during
       the handlers to serialize them. */
    memset(sigact_table, 0, sizeof(sigact_table));

    sigfillset(&act.sa_mask);
    act.sa_flags = SA_SIGINFO;
    act.sa_sigaction = host_signal_handler;
    for(i = 1; i <= TARGET_NSIG; i++) {
        host_sig = target_to_host_signal(i);
        sigaction(host_sig, NULL, &oact);
        if (oact.sa_sigaction == (void *)SIG_IGN) {
            sigact_table[i - 1]._sa_handler = TARGET_SIG_IGN;
        } else if (oact.sa_sigaction == (void *)SIG_DFL) {
            sigact_table[i - 1]._sa_handler = TARGET_SIG_DFL;
        }
        /* If there's already a handler installed then something has
           gone horribly wrong, so don't even try to handle that case.  */
        /* Install some handlers for our own use.  We need at least
           SIGSEGV and SIGBUS, to detect exceptions.  We can not just
           trap all signals because it affects syscall interrupt
           behavior.  But do trap all default-fatal signals.  */
        if (fatal_signal (i))
            sigaction(host_sig, &act, NULL);
    }
}

/* Force a synchronously taken signal. The kernel force_sig() function
 * also forces the signal to "not blocked, not ignored", but for QEMU
 * that work is done in process_pending_signals().
 */
void force_sig(int sig)
{
    CPUState *cpu = thread_cpu;
    CPUArchState *env = cpu->env_ptr;
    target_siginfo_t info;

    info.si_signo = sig;
    info.si_errno = 0;
    info.si_code = TARGET_SI_KERNEL;
    info._sifields._kill._pid = 0;
    info._sifields._kill._uid = 0;
    queue_signal(env, info.si_signo, QEMU_SI_KILL, &info);
}

/* Force a SIGSEGV if we couldn't write to memory trying to set
 * up the signal frame. oldsig is the signal we were trying to handle
 * at the point of failure.
 */
#if !defined(TARGET_RISCV)
void force_sigsegv(int oldsig)
{
    if (oldsig == SIGSEGV) {
        /* Make sure we don't try to deliver the signal again; this will
         * end up with handle_pending_signal() calling dump_core_and_abort().
         */
        sigact_table[oldsig - 1]._sa_handler = TARGET_SIG_DFL;
    }
    force_sig(TARGET_SIGSEGV);
}

#endif

/* abort execution with signal */
static void QEMU_NORETURN dump_core_and_abort(int target_sig)
{
    CPUState *cpu = thread_cpu;
    CPUArchState *env = cpu->env_ptr;
    TaskState *ts = (TaskState *)cpu->opaque;
    int host_sig, core_dumped = 0;
    struct sigaction act;

    host_sig = target_to_host_signal(target_sig);
    trace_user_force_sig(env, target_sig, host_sig);
    gdb_signalled(env, target_sig);

    /* dump core if supported by target binary format */
    if (core_dump_signal(target_sig) && (ts->bprm->core_dump != NULL)) {
        stop_all_tasks();
        core_dumped =
            ((*ts->bprm->core_dump)(target_sig, env) == 0);
    }
    if (core_dumped) {
        /* we already dumped the core of target process, we don't want
         * a coredump of qemu itself */
        struct rlimit nodump;
        getrlimit(RLIMIT_CORE, &nodump);
        nodump.rlim_cur=0;
        setrlimit(RLIMIT_CORE, &nodump);
        (void) fprintf(stderr, "qemu: uncaught target signal %d (%s) - %s\n",
            target_sig, strsignal(host_sig), "core dumped" );
    }

    /* The proper exit code for dying from an uncaught signal is
     * -<signal>.  The kernel doesn't allow exit() or _exit() to pass
     * a negative value.  To get the proper exit code we need to
     * actually die from an uncaught signal.  Here the default signal
     * handler is installed, we send ourself a signal and we wait for
     * it to arrive. */
    sigfillset(&act.sa_mask);
    act.sa_handler = SIG_DFL;
    act.sa_flags = 0;
    sigaction(host_sig, &act, NULL);

    /* For some reason raise(host_sig) doesn't send the signal when
     * statically linked on x86-64. */
    kill(getpid(), host_sig);

    /* Make sure the signal isn't masked (just reuse the mask inside
    of act) */
    sigdelset(&act.sa_mask, host_sig);
    sigsuspend(&act.sa_mask);

    /* unreachable */
    abort();
}

/* queue a signal so that it will be send to the virtual CPU as soon
   as possible */
int queue_signal(CPUArchState *env, int sig, int si_type,
                 target_siginfo_t *info)
{
    CPUState *cpu = ENV_GET_CPU(env);
    TaskState *ts = cpu->opaque;

    trace_user_queue_signal(env, sig);

    info->si_code = deposit32(info->si_code, 16, 16, si_type);

    ts->sync_signal.info = *info;
    ts->sync_signal.pending = sig;
    /* signal that a new signal is pending */
    atomic_set(&ts->signal_pending, 1);
    return 1; /* indicates that the signal was queued */
}

#ifndef HAVE_SAFE_SYSCALL
static inline void rewind_if_in_safe_syscall(void *puc)
{
    /* Default version: never rewind */
}
#endif

static void host_signal_handler(int host_signum, siginfo_t *info,
                                void *puc)
{
    CPUArchState *env = thread_cpu->env_ptr;
    CPUState *cpu = ENV_GET_CPU(env);
    TaskState *ts = cpu->opaque;

    int sig;
    target_siginfo_t tinfo;
    ucontext_t *uc = puc;
    struct emulated_sigtable *k;

    /* the CPU emulator uses some host signals to detect exceptions,
       we forward to it some signals */
    if ((host_signum == SIGSEGV || host_signum == SIGBUS)
        && info->si_code > 0) {
        if (cpu_signal_handler(host_signum, info, puc))
            return;
    }

    /* get target signal number */
    sig = host_to_target_signal(host_signum);
    if (sig < 1 || sig > TARGET_NSIG)
        return;
    trace_user_host_signal(env, host_signum, sig);

    rewind_if_in_safe_syscall(puc);

    host_to_target_siginfo_noswap(&tinfo, info);
    k = &ts->sigtab[sig - 1];
    k->info = tinfo;
    k->pending = sig;
    ts->signal_pending = 1;

    /* Block host signals until target signal handler entered. We
     * can't block SIGSEGV or SIGBUS while we're executing guest
     * code in case the guest code provokes one in the window between
     * now and it getting out to the main loop. Signals will be
     * unblocked again in process_pending_signals().
     *
     * WARNING: we cannot use sigfillset() here because the uc_sigmask
     * field is a kernel sigset_t, which is much smaller than the
     * libc sigset_t which sigfillset() operates on. Using sigfillset()
     * would write 0xff bytes off the end of the structure and trash
     * data on the struct.
     * We can't use sizeof(uc->uc_sigmask) either, because the libc
     * headers define the struct field with the wrong (too large) type.
     */
    memset(&uc->uc_sigmask, 0xff, SIGSET_T_SIZE);
    sigdelset(&uc->uc_sigmask, SIGSEGV);
    sigdelset(&uc->uc_sigmask, SIGBUS);

    /* interrupt the virtual CPU as soon as possible */
    cpu_exit(thread_cpu);
}

/* do_sigaltstack() returns target values and errnos. */
/* compare linux/kernel/signal.c:do_sigaltstack() */
abi_long do_sigaltstack(abi_ulong uss_addr, abi_ulong uoss_addr, abi_ulong sp)
{
    int ret;
    struct target_sigaltstack oss;

    /* XXX: test errors */
    if(uoss_addr)
    {
        __put_user(target_sigaltstack_used.ss_sp, &oss.ss_sp);
        __put_user(target_sigaltstack_used.ss_size, &oss.ss_size);
        __put_user(sas_ss_flags(sp), &oss.ss_flags);
    }

    if(uss_addr)
    {
        struct target_sigaltstack *uss;
        struct target_sigaltstack ss;
        size_t minstacksize = TARGET_MINSIGSTKSZ;

#if defined(TARGET_PPC64)
        /* ELF V2 for PPC64 has a 4K minimum stack size for signal handlers */
        struct image_info *image = ((TaskState *)thread_cpu->opaque)->info;
        if (get_ppc64_abi(image) > 1) {
            minstacksize = 4096;
        }
#endif

	ret = -TARGET_EFAULT;
        if (!lock_user_struct(VERIFY_READ, uss, uss_addr, 1)) {
            goto out;
        }
        __get_user(ss.ss_sp, &uss->ss_sp);
        __get_user(ss.ss_size, &uss->ss_size);
        __get_user(ss.ss_flags, &uss->ss_flags);
        unlock_user_struct(uss, uss_addr, 0);

	ret = -TARGET_EPERM;
	if (on_sig_stack(sp))
            goto out;

	ret = -TARGET_EINVAL;
	if (ss.ss_flags != TARGET_SS_DISABLE
            && ss.ss_flags != TARGET_SS_ONSTACK
            && ss.ss_flags != 0)
            goto out;

	if (ss.ss_flags == TARGET_SS_DISABLE) {
            ss.ss_size = 0;
            ss.ss_sp = 0;
	} else {
            ret = -TARGET_ENOMEM;
            if (ss.ss_size < minstacksize) {
                goto out;
            }
	}

        target_sigaltstack_used.ss_sp = ss.ss_sp;
        target_sigaltstack_used.ss_size = ss.ss_size;
    }

    if (uoss_addr) {
        ret = -TARGET_EFAULT;
        if (copy_to_user(uoss_addr, &oss, sizeof(oss)))
            goto out;
    }

    ret = 0;
out:
    return ret;
}

/* do_sigaction() return target values and host errnos */
int do_sigaction(int sig, const struct target_sigaction *act,
                 struct target_sigaction *oact)
{
    struct target_sigaction *k;
    struct sigaction act1;
    int host_sig;
    int ret = 0;

    if (sig < 1 || sig > TARGET_NSIG || sig == TARGET_SIGKILL || sig == TARGET_SIGSTOP) {
        return -TARGET_EINVAL;
    }

    if (block_signals()) {
        return -TARGET_ERESTARTSYS;
    }

    k = &sigact_table[sig - 1];
    if (oact) {
        __put_user(k->_sa_handler, &oact->_sa_handler);
        __put_user(k->sa_flags, &oact->sa_flags);
#ifdef TARGET_ARCH_HAS_SA_RESTORER
        __put_user(k->sa_restorer, &oact->sa_restorer);
#endif
        /* Not swapped.  */
        oact->sa_mask = k->sa_mask;
    }
    if (act) {
        /* FIXME: This is not threadsafe.  */
        __get_user(k->_sa_handler, &act->_sa_handler);
        __get_user(k->sa_flags, &act->sa_flags);
#ifdef TARGET_ARCH_HAS_SA_RESTORER
        __get_user(k->sa_restorer, &act->sa_restorer);
#endif
        /* To be swapped in target_to_host_sigset.  */
        k->sa_mask = act->sa_mask;

        /* we update the host linux signal state */
        host_sig = target_to_host_signal(sig);
        if (host_sig != SIGSEGV && host_sig != SIGBUS) {
            sigfillset(&act1.sa_mask);
            act1.sa_flags = SA_SIGINFO;
            if (k->sa_flags & TARGET_SA_RESTART)
                act1.sa_flags |= SA_RESTART;
            /* NOTE: it is important to update the host kernel signal
               ignore state to avoid getting unexpected interrupted
               syscalls */
            if (k->_sa_handler == TARGET_SIG_IGN) {
                act1.sa_sigaction = (void *)SIG_IGN;
            } else if (k->_sa_handler == TARGET_SIG_DFL) {
                if (fatal_signal (sig))
                    act1.sa_sigaction = host_signal_handler;
                else
                    act1.sa_sigaction = (void *)SIG_DFL;
            } else {
                act1.sa_sigaction = host_signal_handler;
            }
            ret = sigaction(host_sig, &act1, NULL);
        }
    }
    return ret;
}

#if defined(TARGET_I386)
/* from the Linux kernel - /arch/x86/include/uapi/asm/sigcontext.h */

struct target_fpreg {
    uint16_t significand[4];
    uint16_t exponent;
};

struct target_fpxreg {
    uint16_t significand[4];
    uint16_t exponent;
    uint16_t padding[3];
};

struct target_xmmreg {
    uint32_t element[4];
};

struct target_fpstate_32 {
    /* Regular FPU environment */
    uint32_t cw;
    uint32_t sw;
    uint32_t tag;
    uint32_t ipoff;
    uint32_t cssel;
    uint32_t dataoff;
    uint32_t datasel;
    struct target_fpreg st[8];
    uint16_t  status;
    uint16_t  magic;          /* 0xffff = regular FPU data only */

    /* FXSR FPU environment */
    uint32_t _fxsr_env[6];   /* FXSR FPU env is ignored */
    uint32_t mxcsr;
    uint32_t reserved;
    struct target_fpxreg fxsr_st[8]; /* FXSR FPU reg data is ignored */
    struct target_xmmreg xmm[8];
    uint32_t padding[56];
};

struct target_fpstate_64 {
    /* FXSAVE format */
    uint16_t cw;
    uint16_t sw;
    uint16_t twd;
    uint16_t fop;
    uint64_t rip;
    uint64_t rdp;
    uint32_t mxcsr;
    uint32_t mxcsr_mask;
    uint32_t st_space[32];
    uint32_t xmm_space[64];
    uint32_t reserved[24];
};

#ifndef TARGET_X86_64
# define target_fpstate target_fpstate_32
#else
# define target_fpstate target_fpstate_64
#endif

struct target_sigcontext_32 {
    uint16_t gs, __gsh;
    uint16_t fs, __fsh;
    uint16_t es, __esh;
    uint16_t ds, __dsh;
    uint32_t edi;
    uint32_t esi;
    uint32_t ebp;
    uint32_t esp;
    uint32_t ebx;
    uint32_t edx;
    uint32_t ecx;
    uint32_t eax;
    uint32_t trapno;
    uint32_t err;
    uint32_t eip;
    uint16_t cs, __csh;
    uint32_t eflags;
    uint32_t esp_at_signal;
    uint16_t ss, __ssh;
    uint32_t fpstate; /* pointer */
    uint32_t oldmask;
    uint32_t cr2;
};

struct target_sigcontext_64 {
    uint64_t r8;
    uint64_t r9;
    uint64_t r10;
    uint64_t r11;
    uint64_t r12;
    uint64_t r13;
    uint64_t r14;
    uint64_t r15;

    uint64_t rdi;
    uint64_t rsi;
    uint64_t rbp;
    uint64_t rbx;
    uint64_t rdx;
    uint64_t rax;
    uint64_t rcx;
    uint64_t rsp;
    uint64_t rip;

    uint64_t eflags;

    uint16_t cs;
    uint16_t gs;
    uint16_t fs;
    uint16_t ss;

    uint64_t err;
    uint64_t trapno;
    uint64_t oldmask;
    uint64_t cr2;

    uint64_t fpstate; /* pointer */
    uint64_t padding[8];
};

#ifndef TARGET_X86_64
# define target_sigcontext target_sigcontext_32
#else
# define target_sigcontext target_sigcontext_64
#endif

/* see Linux/include/uapi/asm-generic/ucontext.h */
struct target_ucontext {
    abi_ulong         tuc_flags;
    abi_ulong         tuc_link;
    target_stack_t    tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t   tuc_sigmask;  /* mask last for extensibility */
};

#ifndef TARGET_X86_64
struct sigframe {
    abi_ulong pretcode;
    int sig;
    struct target_sigcontext sc;
    struct target_fpstate fpstate;
    abi_ulong extramask[TARGET_NSIG_WORDS-1];
    char retcode[8];
};

struct rt_sigframe {
    abi_ulong pretcode;
    int sig;
    abi_ulong pinfo;
    abi_ulong puc;
    struct target_siginfo info;
    struct target_ucontext uc;
    struct target_fpstate fpstate;
    char retcode[8];
};

#else

struct rt_sigframe {
    abi_ulong pretcode;
    struct target_ucontext uc;
    struct target_siginfo info;
    struct target_fpstate fpstate;
};

#endif

/*
 * Set up a signal frame.
 */

/* XXX: save x87 state */
static void setup_sigcontext(struct target_sigcontext *sc,
        struct target_fpstate *fpstate, CPUX86State *env, abi_ulong mask,
        abi_ulong fpstate_addr)
{
    CPUState *cs = CPU(x86_env_get_cpu(env));
#ifndef TARGET_X86_64
    uint16_t magic;

    /* already locked in setup_frame() */
    __put_user(env->segs[R_GS].selector, (unsigned int *)&sc->gs);
    __put_user(env->segs[R_FS].selector, (unsigned int *)&sc->fs);
    __put_user(env->segs[R_ES].selector, (unsigned int *)&sc->es);
    __put_user(env->segs[R_DS].selector, (unsigned int *)&sc->ds);
    __put_user(env->regs[R_EDI], &sc->edi);
    __put_user(env->regs[R_ESI], &sc->esi);
    __put_user(env->regs[R_EBP], &sc->ebp);
    __put_user(env->regs[R_ESP], &sc->esp);
    __put_user(env->regs[R_EBX], &sc->ebx);
    __put_user(env->regs[R_EDX], &sc->edx);
    __put_user(env->regs[R_ECX], &sc->ecx);
    __put_user(env->regs[R_EAX], &sc->eax);
    __put_user(cs->exception_index, &sc->trapno);
    __put_user(env->error_code, &sc->err);
    __put_user(env->eip, &sc->eip);
    __put_user(env->segs[R_CS].selector, (unsigned int *)&sc->cs);
    __put_user(env->eflags, &sc->eflags);
    __put_user(env->regs[R_ESP], &sc->esp_at_signal);
    __put_user(env->segs[R_SS].selector, (unsigned int *)&sc->ss);

    cpu_x86_fsave(env, fpstate_addr, 1);
    fpstate->status = fpstate->sw;
    magic = 0xffff;
    __put_user(magic, &fpstate->magic);
    __put_user(fpstate_addr, &sc->fpstate);

    /* non-iBCS2 extensions.. */
    __put_user(mask, &sc->oldmask);
    __put_user(env->cr[2], &sc->cr2);
#else
    __put_user(env->regs[R_EDI], &sc->rdi);
    __put_user(env->regs[R_ESI], &sc->rsi);
    __put_user(env->regs[R_EBP], &sc->rbp);
    __put_user(env->regs[R_ESP], &sc->rsp);
    __put_user(env->regs[R_EBX], &sc->rbx);
    __put_user(env->regs[R_EDX], &sc->rdx);
    __put_user(env->regs[R_ECX], &sc->rcx);
    __put_user(env->regs[R_EAX], &sc->rax);

    __put_user(env->regs[8], &sc->r8);
    __put_user(env->regs[9], &sc->r9);
    __put_user(env->regs[10], &sc->r10);
    __put_user(env->regs[11], &sc->r11);
    __put_user(env->regs[12], &sc->r12);
    __put_user(env->regs[13], &sc->r13);
    __put_user(env->regs[14], &sc->r14);
    __put_user(env->regs[15], &sc->r15);

    __put_user(cs->exception_index, &sc->trapno);
    __put_user(env->error_code, &sc->err);
    __put_user(env->eip, &sc->rip);

    __put_user(env->eflags, &sc->eflags);
    __put_user(env->segs[R_CS].selector, &sc->cs);
    __put_user((uint16_t)0, &sc->gs);
    __put_user((uint16_t)0, &sc->fs);
    __put_user(env->segs[R_SS].selector, &sc->ss);

    __put_user(mask, &sc->oldmask);
    __put_user(env->cr[2], &sc->cr2);

    /* fpstate_addr must be 16 byte aligned for fxsave */
    assert(!(fpstate_addr & 0xf));

    cpu_x86_fxsave(env, fpstate_addr);
    __put_user(fpstate_addr, &sc->fpstate);
#endif
}

/*
 * Determine which stack to use..
 */

static inline abi_ulong
get_sigframe(struct target_sigaction *ka, CPUX86State *env, size_t frame_size)
{
    unsigned long esp;

    /* Default to using normal stack */
    esp = env->regs[R_ESP];
#ifdef TARGET_X86_64
    esp -= 128; /* this is the redzone */
#endif

    /* This is the X/Open sanctioned signal stack switching.  */
    if (ka->sa_flags & TARGET_SA_ONSTACK) {
        if (sas_ss_flags(esp) == 0) {
            esp = target_sigaltstack_used.ss_sp + target_sigaltstack_used.ss_size;
        }
    } else {
#ifndef TARGET_X86_64
        /* This is the legacy signal stack switching. */
        if ((env->segs[R_SS].selector & 0xffff) != __USER_DS &&
                !(ka->sa_flags & TARGET_SA_RESTORER) &&
                ka->sa_restorer) {
            esp = (unsigned long) ka->sa_restorer;
        }
#endif
    }

#ifndef TARGET_X86_64
    return (esp - frame_size) & -8ul;
#else
    return ((esp - frame_size) & (~15ul)) - 8;
#endif
}

#ifndef TARGET_X86_64
/* compare linux/arch/i386/kernel/signal.c:setup_frame() */
static void setup_frame(int sig, struct target_sigaction *ka,
                        target_sigset_t *set, CPUX86State *env)
{
    abi_ulong frame_addr;
    struct sigframe *frame;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_frame(env, frame_addr);

    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0))
        goto give_sigsegv;

    __put_user(sig, &frame->sig);

    setup_sigcontext(&frame->sc, &frame->fpstate, env, set->sig[0],
            frame_addr + offsetof(struct sigframe, fpstate));

    for(i = 1; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->extramask[i - 1]);
    }

    /* Set up to return from userspace.  If provided, use a stub
       already in userspace.  */
    if (ka->sa_flags & TARGET_SA_RESTORER) {
        __put_user(ka->sa_restorer, &frame->pretcode);
    } else {
        uint16_t val16;
        abi_ulong retcode_addr;
        retcode_addr = frame_addr + offsetof(struct sigframe, retcode);
        __put_user(retcode_addr, &frame->pretcode);
        /* This is popl %eax ; movl $,%eax ; int $0x80 */
        val16 = 0xb858;
        __put_user(val16, (uint16_t *)(frame->retcode+0));
        __put_user(TARGET_NR_sigreturn, (int *)(frame->retcode+2));
        val16 = 0x80cd;
        __put_user(val16, (uint16_t *)(frame->retcode+6));
    }

    /* Set up registers for signal handler */
    env->regs[R_ESP] = frame_addr;
    env->eip = ka->_sa_handler;

    cpu_x86_load_seg(env, R_DS, __USER_DS);
    cpu_x86_load_seg(env, R_ES, __USER_DS);
    cpu_x86_load_seg(env, R_SS, __USER_DS);
    cpu_x86_load_seg(env, R_CS, __USER_CS);
    env->eflags &= ~TF_MASK;

    unlock_user_struct(frame, frame_addr, 1);

    return;

give_sigsegv:
    force_sigsegv(sig);
}
#endif

/* compare linux/arch/x86/kernel/signal.c:setup_rt_frame() */
static void setup_rt_frame(int sig, struct target_sigaction *ka,
                           target_siginfo_t *info,
                           target_sigset_t *set, CPUX86State *env)
{
    abi_ulong frame_addr;
#ifndef TARGET_X86_64
    abi_ulong addr;
#endif
    struct rt_sigframe *frame;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);

    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0))
        goto give_sigsegv;

    /* These fields are only in rt_sigframe on 32 bit */
#ifndef TARGET_X86_64
    __put_user(sig, &frame->sig);
    addr = frame_addr + offsetof(struct rt_sigframe, info);
    __put_user(addr, &frame->pinfo);
    addr = frame_addr + offsetof(struct rt_sigframe, uc);
    __put_user(addr, &frame->puc);
#endif
    if (ka->sa_flags & TARGET_SA_SIGINFO) {
        tswap_siginfo(&frame->info, info);
    }

    /* Create the ucontext.  */
    __put_user(0, &frame->uc.tuc_flags);
    __put_user(0, &frame->uc.tuc_link);
    __put_user(target_sigaltstack_used.ss_sp, &frame->uc.tuc_stack.ss_sp);
    __put_user(sas_ss_flags(get_sp_from_cpustate(env)),
               &frame->uc.tuc_stack.ss_flags);
    __put_user(target_sigaltstack_used.ss_size,
               &frame->uc.tuc_stack.ss_size);
    setup_sigcontext(&frame->uc.tuc_mcontext, &frame->fpstate, env,
            set->sig[0], frame_addr + offsetof(struct rt_sigframe, fpstate));

    for(i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }

    /* Set up to return from userspace.  If provided, use a stub
       already in userspace.  */
#ifndef TARGET_X86_64
    if (ka->sa_flags & TARGET_SA_RESTORER) {
        __put_user(ka->sa_restorer, &frame->pretcode);
    } else {
        uint16_t val16;
        addr = frame_addr + offsetof(struct rt_sigframe, retcode);
        __put_user(addr, &frame->pretcode);
        /* This is movl $,%eax ; int $0x80 */
        __put_user(0xb8, (char *)(frame->retcode+0));
        __put_user(TARGET_NR_rt_sigreturn, (int *)(frame->retcode+1));
        val16 = 0x80cd;
        __put_user(val16, (uint16_t *)(frame->retcode+5));
    }
#else
    /* XXX: Would be slightly better to return -EFAULT here if test fails
       assert(ka->sa_flags & TARGET_SA_RESTORER); */
    __put_user(ka->sa_restorer, &frame->pretcode);
#endif

    /* Set up registers for signal handler */
    env->regs[R_ESP] = frame_addr;
    env->eip = ka->_sa_handler;

#ifndef TARGET_X86_64
    env->regs[R_EAX] = sig;
    env->regs[R_EDX] = (unsigned long)&frame->info;
    env->regs[R_ECX] = (unsigned long)&frame->uc;
#else
    env->regs[R_EAX] = 0;
    env->regs[R_EDI] = sig;
    env->regs[R_ESI] = (unsigned long)&frame->info;
    env->regs[R_EDX] = (unsigned long)&frame->uc;
#endif

    cpu_x86_load_seg(env, R_DS, __USER_DS);
    cpu_x86_load_seg(env, R_ES, __USER_DS);
    cpu_x86_load_seg(env, R_CS, __USER_CS);
    cpu_x86_load_seg(env, R_SS, __USER_DS);
    env->eflags &= ~TF_MASK;

    unlock_user_struct(frame, frame_addr, 1);

    return;

give_sigsegv:
    force_sigsegv(sig);
}

static int
restore_sigcontext(CPUX86State *env, struct target_sigcontext *sc)
{
    unsigned int err = 0;
    abi_ulong fpstate_addr;
    unsigned int tmpflags;

#ifndef TARGET_X86_64
    cpu_x86_load_seg(env, R_GS, tswap16(sc->gs));
    cpu_x86_load_seg(env, R_FS, tswap16(sc->fs));
    cpu_x86_load_seg(env, R_ES, tswap16(sc->es));
    cpu_x86_load_seg(env, R_DS, tswap16(sc->ds));

    env->regs[R_EDI] = tswapl(sc->edi);
    env->regs[R_ESI] = tswapl(sc->esi);
    env->regs[R_EBP] = tswapl(sc->ebp);
    env->regs[R_ESP] = tswapl(sc->esp);
    env->regs[R_EBX] = tswapl(sc->ebx);
    env->regs[R_EDX] = tswapl(sc->edx);
    env->regs[R_ECX] = tswapl(sc->ecx);
    env->regs[R_EAX] = tswapl(sc->eax);

    env->eip = tswapl(sc->eip);
#else
    env->regs[8] = tswapl(sc->r8);
    env->regs[9] = tswapl(sc->r9);
    env->regs[10] = tswapl(sc->r10);
    env->regs[11] = tswapl(sc->r11);
    env->regs[12] = tswapl(sc->r12);
    env->regs[13] = tswapl(sc->r13);
    env->regs[14] = tswapl(sc->r14);
    env->regs[15] = tswapl(sc->r15);

    env->regs[R_EDI] = tswapl(sc->rdi);
    env->regs[R_ESI] = tswapl(sc->rsi);
    env->regs[R_EBP] = tswapl(sc->rbp);
    env->regs[R_EBX] = tswapl(sc->rbx);
    env->regs[R_EDX] = tswapl(sc->rdx);
    env->regs[R_EAX] = tswapl(sc->rax);
    env->regs[R_ECX] = tswapl(sc->rcx);
    env->regs[R_ESP] = tswapl(sc->rsp);

    env->eip = tswapl(sc->rip);
#endif

    cpu_x86_load_seg(env, R_CS, lduw_p(&sc->cs) | 3);
    cpu_x86_load_seg(env, R_SS, lduw_p(&sc->ss) | 3);

    tmpflags = tswapl(sc->eflags);
    env->eflags = (env->eflags & ~0x40DD5) | (tmpflags & 0x40DD5);
    //		regs->orig_eax = -1;		/* disable syscall checks */

    fpstate_addr = tswapl(sc->fpstate);
    if (fpstate_addr != 0) {
        if (!access_ok(VERIFY_READ, fpstate_addr,
                       sizeof(struct target_fpstate)))
            goto badframe;
#ifndef TARGET_X86_64
        cpu_x86_frstor(env, fpstate_addr, 1);
#else
        cpu_x86_fxrstor(env, fpstate_addr);
#endif
    }

    return err;
badframe:
    return 1;
}

/* Note: there is no sigreturn on x86_64, there is only rt_sigreturn */
#ifndef TARGET_X86_64
long do_sigreturn(CPUX86State *env)
{
    struct sigframe *frame;
    abi_ulong frame_addr = env->regs[R_ESP] - 8;
    target_sigset_t target_set;
    sigset_t set;
    int i;

    trace_user_do_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1))
        goto badframe;
    /* set blocked signals */
    __get_user(target_set.sig[0], &frame->sc.oldmask);
    for(i = 1; i < TARGET_NSIG_WORDS; i++) {
        __get_user(target_set.sig[i], &frame->extramask[i - 1]);
    }

    target_to_host_sigset_internal(&set, &target_set);
    set_sigmask(&set);

    /* restore registers */
    if (restore_sigcontext(env, &frame->sc))
        goto badframe;
    unlock_user_struct(frame, frame_addr, 0);
    return -TARGET_QEMU_ESIGRETURN;

badframe:
    unlock_user_struct(frame, frame_addr, 0);
    force_sig(TARGET_SIGSEGV);
    return -TARGET_QEMU_ESIGRETURN;
}
#endif

long do_rt_sigreturn(CPUX86State *env)
{
    abi_ulong frame_addr;
    struct rt_sigframe *frame;
    sigset_t set;

    frame_addr = env->regs[R_ESP] - sizeof(abi_ulong);
    trace_user_do_rt_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1))
        goto badframe;
    target_to_host_sigset(&set, &frame->uc.tuc_sigmask);
    set_sigmask(&set);

    if (restore_sigcontext(env, &frame->uc.tuc_mcontext)) {
        goto badframe;
    }

    if (do_sigaltstack(frame_addr + offsetof(struct rt_sigframe, uc.tuc_stack), 0,
                       get_sp_from_cpustate(env)) == -EFAULT) {
        goto badframe;
    }

    unlock_user_struct(frame, frame_addr, 0);
    return -TARGET_QEMU_ESIGRETURN;

badframe:
    unlock_user_struct(frame, frame_addr, 0);
    force_sig(TARGET_SIGSEGV);
    return -TARGET_QEMU_ESIGRETURN;
}

#elif defined(TARGET_SPARC)

#define __SUNOS_MAXWIN   31

/* This is what SunOS does, so shall I. */
struct target_sigcontext {
    abi_ulong sigc_onstack;      /* state to restore */

    abi_ulong sigc_mask;         /* sigmask to restore */
    abi_ulong sigc_sp;           /* stack pointer */
    abi_ulong sigc_pc;           /* program counter */
    abi_ulong sigc_npc;          /* next program counter */
    abi_ulong sigc_psr;          /* for condition codes etc */
    abi_ulong sigc_g1;           /* User uses these two registers */
    abi_ulong sigc_o0;           /* within the trampoline code. */

    /* Now comes information regarding the users window set
         * at the time of the signal.
         */
    abi_ulong sigc_oswins;       /* outstanding windows */

    /* stack ptrs for each regwin buf */
    char *sigc_spbuf[__SUNOS_MAXWIN];

    /* Windows to restore after signal */
    struct {
        abi_ulong locals[8];
        abi_ulong ins[8];
    } sigc_wbuf[__SUNOS_MAXWIN];
};
/* A Sparc stack frame */
struct sparc_stackf {
    abi_ulong locals[8];
    abi_ulong ins[8];
    /* It's simpler to treat fp and callers_pc as elements of ins[]
         * since we never need to access them ourselves.
         */
    char *structptr;
    abi_ulong xargs[6];
    abi_ulong xxargs[1];
};

typedef struct {
    struct {
        abi_ulong psr;
        abi_ulong pc;
        abi_ulong npc;
        abi_ulong y;
        abi_ulong u_regs[16]; /* globals and ins */
    }               si_regs;
    int             si_mask;
} __siginfo_t;

typedef struct {
    abi_ulong  si_float_regs[32];
    unsigned   long si_fsr;
    unsigned   long si_fpqdepth;
    struct {
        unsigned long *insn_addr;
        unsigned long insn;
    } si_fpqueue [16];
} qemu_siginfo_fpu_t;


struct target_signal_frame {
    struct sparc_stackf ss;
    __siginfo_t         info;
    abi_ulong           fpu_save;
    abi_ulong           insns[2] __attribute__ ((aligned (8)));
    abi_ulong           extramask[TARGET_NSIG_WORDS - 1];
    abi_ulong           extra_size; /* Should be 0 */
    qemu_siginfo_fpu_t fpu_state;
};
struct target_rt_signal_frame {
    struct sparc_stackf ss;
    siginfo_t           info;
    abi_ulong           regs[20];
    sigset_t            mask;
    abi_ulong           fpu_save;
    unsigned int        insns[2];
    stack_t             stack;
    unsigned int        extra_size; /* Should be 0 */
    qemu_siginfo_fpu_t  fpu_state;
};

#define UREG_O0        16
#define UREG_O6        22
#define UREG_I0        0
#define UREG_I1        1
#define UREG_I2        2
#define UREG_I3        3
#define UREG_I4        4
#define UREG_I5        5
#define UREG_I6        6
#define UREG_I7        7
#define UREG_L0	       8
#define UREG_FP        UREG_I6
#define UREG_SP        UREG_O6

static inline abi_ulong get_sigframe(struct target_sigaction *sa, 
                                     CPUSPARCState *env,
                                     unsigned long framesize)
{
    abi_ulong sp;

    sp = env->regwptr[UREG_FP];

    /* This is the X/Open sanctioned signal stack switching.  */
    if (sa->sa_flags & TARGET_SA_ONSTACK) {
        if (!on_sig_stack(sp)
                && !((target_sigaltstack_used.ss_sp + target_sigaltstack_used.ss_size) & 7)) {
            sp = target_sigaltstack_used.ss_sp + target_sigaltstack_used.ss_size;
        }
    }
    return sp - framesize;
}

static int
setup___siginfo(__siginfo_t *si, CPUSPARCState *env, abi_ulong mask)
{
    int err = 0, i;

    __put_user(env->psr, &si->si_regs.psr);
    __put_user(env->pc, &si->si_regs.pc);
    __put_user(env->npc, &si->si_regs.npc);
    __put_user(env->y, &si->si_regs.y);
    for (i=0; i < 8; i++) {
        __put_user(env->gregs[i], &si->si_regs.u_regs[i]);
    }
    for (i=0; i < 8; i++) {
        __put_user(env->regwptr[UREG_I0 + i], &si->si_regs.u_regs[i+8]);
    }
    __put_user(mask, &si->si_mask);
    return err;
}

#if 0
static int
setup_sigcontext(struct target_sigcontext *sc, /*struct _fpstate *fpstate,*/
                 CPUSPARCState *env, unsigned long mask)
{
    int err = 0;

    __put_user(mask, &sc->sigc_mask);
    __put_user(env->regwptr[UREG_SP], &sc->sigc_sp);
    __put_user(env->pc, &sc->sigc_pc);
    __put_user(env->npc, &sc->sigc_npc);
    __put_user(env->psr, &sc->sigc_psr);
    __put_user(env->gregs[1], &sc->sigc_g1);
    __put_user(env->regwptr[UREG_O0], &sc->sigc_o0);

    return err;
}
#endif
#define NF_ALIGNEDSZ  (((sizeof(struct target_signal_frame) + 7) & (~7)))

static void setup_frame(int sig, struct target_sigaction *ka,
                        target_sigset_t *set, CPUSPARCState *env)
{
    abi_ulong sf_addr;
    struct target_signal_frame *sf;
    int sigframe_size, err, i;

    /* 1. Make sure everything is clean */
    //synchronize_user_stack();

    sigframe_size = NF_ALIGNEDSZ;
    sf_addr = get_sigframe(ka, env, sigframe_size);
    trace_user_setup_frame(env, sf_addr);

    sf = lock_user(VERIFY_WRITE, sf_addr,
                   sizeof(struct target_signal_frame), 0);
    if (!sf) {
        goto sigsegv;
    }
#if 0
    if (invalid_frame_pointer(sf, sigframe_size))
        goto sigill_and_return;
#endif
    /* 2. Save the current process state */
    err = setup___siginfo(&sf->info, env, set->sig[0]);
    __put_user(0, &sf->extra_size);

    //save_fpu_state(regs, &sf->fpu_state);
    //__put_user(&sf->fpu_state, &sf->fpu_save);

    __put_user(set->sig[0], &sf->info.si_mask);
    for (i = 0; i < TARGET_NSIG_WORDS - 1; i++) {
        __put_user(set->sig[i + 1], &sf->extramask[i]);
    }

    for (i = 0; i < 8; i++) {
        __put_user(env->regwptr[i + UREG_L0], &sf->ss.locals[i]);
    }
    for (i = 0; i < 8; i++) {
        __put_user(env->regwptr[i + UREG_I0], &sf->ss.ins[i]);
    }
    if (err)
        goto sigsegv;

    /* 3. signal handler back-trampoline and parameters */
    env->regwptr[UREG_FP] = sf_addr;
    env->regwptr[UREG_I0] = sig;
    env->regwptr[UREG_I1] = sf_addr +
            offsetof(struct target_signal_frame, info);
    env->regwptr[UREG_I2] = sf_addr +
            offsetof(struct target_signal_frame, info);

    /* 4. signal handler */
    env->pc = ka->_sa_handler;
    env->npc = (env->pc + 4);
    /* 5. return to kernel instructions */
    if (ka->ka_restorer) {
        env->regwptr[UREG_I7] = ka->ka_restorer;
    } else {
        uint32_t val32;

        env->regwptr[UREG_I7] = sf_addr +
                offsetof(struct target_signal_frame, insns) - 2 * 4;

        /* mov __NR_sigreturn, %g1 */
        val32 = 0x821020d8;
        __put_user(val32, &sf->insns[0]);

        /* t 0x10 */
        val32 = 0x91d02010;
        __put_user(val32, &sf->insns[1]);
        if (err)
            goto sigsegv;

        /* Flush instruction space. */
        // flush_sig_insns(current->mm, (unsigned long) &(sf->insns[0]));
        // tb_flush(env);
    }
    unlock_user(sf, sf_addr, sizeof(struct target_signal_frame));
    return;
#if 0
sigill_and_return:
    force_sig(TARGET_SIGILL);
#endif
sigsegv:
    unlock_user(sf, sf_addr, sizeof(struct target_signal_frame));
    force_sigsegv(sig);
}

static void setup_rt_frame(int sig, struct target_sigaction *ka,
                           target_siginfo_t *info,
                           target_sigset_t *set, CPUSPARCState *env)
{
    fprintf(stderr, "setup_rt_frame: not implemented\n");
}

long do_sigreturn(CPUSPARCState *env)
{
    abi_ulong sf_addr;
    struct target_signal_frame *sf;
    uint32_t up_psr, pc, npc;
    target_sigset_t set;
    sigset_t host_set;
    int err=0, i;

    sf_addr = env->regwptr[UREG_FP];
    trace_user_do_sigreturn(env, sf_addr);
    if (!lock_user_struct(VERIFY_READ, sf, sf_addr, 1)) {
        goto segv_and_exit;
    }

    /* 1. Make sure we are not getting garbage from the user */

    if (sf_addr & 3)
        goto segv_and_exit;

    __get_user(pc,  &sf->info.si_regs.pc);
    __get_user(npc, &sf->info.si_regs.npc);

    if ((pc | npc) & 3) {
        goto segv_and_exit;
    }

    /* 2. Restore the state */
    __get_user(up_psr, &sf->info.si_regs.psr);

    /* User can only change condition codes and FPU enabling in %psr. */
    env->psr = (up_psr & (PSR_ICC /* | PSR_EF */))
            | (env->psr & ~(PSR_ICC /* | PSR_EF */));

    env->pc = pc;
    env->npc = npc;
    __get_user(env->y, &sf->info.si_regs.y);
    for (i=0; i < 8; i++) {
        __get_user(env->gregs[i], &sf->info.si_regs.u_regs[i]);
    }
    for (i=0; i < 8; i++) {
        __get_user(env->regwptr[i + UREG_I0], &sf->info.si_regs.u_regs[i+8]);
    }

    /* FIXME: implement FPU save/restore:
         * __get_user(fpu_save, &sf->fpu_save);
         * if (fpu_save)
         *        err |= restore_fpu_state(env, fpu_save);
         */

    /* This is pretty much atomic, no amount locking would prevent
         * the races which exist anyways.
         */
    __get_user(set.sig[0], &sf->info.si_mask);
    for(i = 1; i < TARGET_NSIG_WORDS; i++) {
        __get_user(set.sig[i], &sf->extramask[i - 1]);
    }

    target_to_host_sigset_internal(&host_set, &set);
    set_sigmask(&host_set);

    if (err) {
        goto segv_and_exit;
    }
    unlock_user_struct(sf, sf_addr, 0);
    return -TARGET_QEMU_ESIGRETURN;

segv_and_exit:
    unlock_user_struct(sf, sf_addr, 0);
    force_sig(TARGET_SIGSEGV);
    return -TARGET_QEMU_ESIGRETURN;
}

long do_rt_sigreturn(CPUSPARCState *env)
{
    trace_user_do_rt_sigreturn(env, 0);
    fprintf(stderr, "do_rt_sigreturn: not implemented\n");
    return -TARGET_ENOSYS;
}

#if defined(TARGET_SPARC64) && !defined(TARGET_ABI32)
#define SPARC_MC_TSTATE 0
#define SPARC_MC_PC 1
#define SPARC_MC_NPC 2
#define SPARC_MC_Y 3
#define SPARC_MC_G1 4
#define SPARC_MC_G2 5
#define SPARC_MC_G3 6
#define SPARC_MC_G4 7
#define SPARC_MC_G5 8
#define SPARC_MC_G6 9
#define SPARC_MC_G7 10
#define SPARC_MC_O0 11
#define SPARC_MC_O1 12
#define SPARC_MC_O2 13
#define SPARC_MC_O3 14
#define SPARC_MC_O4 15
#define SPARC_MC_O5 16
#define SPARC_MC_O6 17
#define SPARC_MC_O7 18
#define SPARC_MC_NGREG 19

typedef abi_ulong target_mc_greg_t;
typedef target_mc_greg_t target_mc_gregset_t[SPARC_MC_NGREG];

struct target_mc_fq {
    abi_ulong *mcfq_addr;
    uint32_t mcfq_insn;
};

struct target_mc_fpu {
    union {
        uint32_t sregs[32];
        uint64_t dregs[32];
        //uint128_t qregs[16];
    } mcfpu_fregs;
    abi_ulong mcfpu_fsr;
    abi_ulong mcfpu_fprs;
    abi_ulong mcfpu_gsr;
    struct target_mc_fq *mcfpu_fq;
    unsigned char mcfpu_qcnt;
    unsigned char mcfpu_qentsz;
    unsigned char mcfpu_enab;
};
typedef struct target_mc_fpu target_mc_fpu_t;

typedef struct {
    target_mc_gregset_t mc_gregs;
    target_mc_greg_t mc_fp;
    target_mc_greg_t mc_i7;
    target_mc_fpu_t mc_fpregs;
} target_mcontext_t;

struct target_ucontext {
    struct target_ucontext *tuc_link;
    abi_ulong tuc_flags;
    target_sigset_t tuc_sigmask;
    target_mcontext_t tuc_mcontext;
};

/* A V9 register window */
struct target_reg_window {
    abi_ulong locals[8];
    abi_ulong ins[8];
};

#define TARGET_STACK_BIAS 2047

/* {set, get}context() needed for 64-bit SparcLinux userland. */
void sparc64_set_context(CPUSPARCState *env)
{
    abi_ulong ucp_addr;
    struct target_ucontext *ucp;
    target_mc_gregset_t *grp;
    abi_ulong pc, npc, tstate;
    abi_ulong fp, i7, w_addr;
    unsigned int i;

    ucp_addr = env->regwptr[UREG_I0];
    if (!lock_user_struct(VERIFY_READ, ucp, ucp_addr, 1)) {
        goto do_sigsegv;
    }
    grp  = &ucp->tuc_mcontext.mc_gregs;
    __get_user(pc, &((*grp)[SPARC_MC_PC]));
    __get_user(npc, &((*grp)[SPARC_MC_NPC]));
    if ((pc | npc) & 3) {
        goto do_sigsegv;
    }
    if (env->regwptr[UREG_I1]) {
        target_sigset_t target_set;
        sigset_t set;

        if (TARGET_NSIG_WORDS == 1) {
            __get_user(target_set.sig[0], &ucp->tuc_sigmask.sig[0]);
        } else {
            abi_ulong *src, *dst;
            src = ucp->tuc_sigmask.sig;
            dst = target_set.sig;
            for (i = 0; i < TARGET_NSIG_WORDS; i++, dst++, src++) {
                __get_user(*dst, src);
            }
        }
        target_to_host_sigset_internal(&set, &target_set);
        set_sigmask(&set);
    }
    env->pc = pc;
    env->npc = npc;
    __get_user(env->y, &((*grp)[SPARC_MC_Y]));
    __get_user(tstate, &((*grp)[SPARC_MC_TSTATE]));
    env->asi = (tstate >> 24) & 0xff;
    cpu_put_ccr(env, tstate >> 32);
    cpu_put_cwp64(env, tstate & 0x1f);
    __get_user(env->gregs[1], (&(*grp)[SPARC_MC_G1]));
    __get_user(env->gregs[2], (&(*grp)[SPARC_MC_G2]));
    __get_user(env->gregs[3], (&(*grp)[SPARC_MC_G3]));
    __get_user(env->gregs[4], (&(*grp)[SPARC_MC_G4]));
    __get_user(env->gregs[5], (&(*grp)[SPARC_MC_G5]));
    __get_user(env->gregs[6], (&(*grp)[SPARC_MC_G6]));
    __get_user(env->gregs[7], (&(*grp)[SPARC_MC_G7]));
    __get_user(env->regwptr[UREG_I0], (&(*grp)[SPARC_MC_O0]));
    __get_user(env->regwptr[UREG_I1], (&(*grp)[SPARC_MC_O1]));
    __get_user(env->regwptr[UREG_I2], (&(*grp)[SPARC_MC_O2]));
    __get_user(env->regwptr[UREG_I3], (&(*grp)[SPARC_MC_O3]));
    __get_user(env->regwptr[UREG_I4], (&(*grp)[SPARC_MC_O4]));
    __get_user(env->regwptr[UREG_I5], (&(*grp)[SPARC_MC_O5]));
    __get_user(env->regwptr[UREG_I6], (&(*grp)[SPARC_MC_O6]));
    __get_user(env->regwptr[UREG_I7], (&(*grp)[SPARC_MC_O7]));

    __get_user(fp, &(ucp->tuc_mcontext.mc_fp));
    __get_user(i7, &(ucp->tuc_mcontext.mc_i7));

    w_addr = TARGET_STACK_BIAS+env->regwptr[UREG_I6];
    if (put_user(fp, w_addr + offsetof(struct target_reg_window, ins[6]),
                 abi_ulong) != 0) {
        goto do_sigsegv;
    }
    if (put_user(i7, w_addr + offsetof(struct target_reg_window, ins[7]),
                 abi_ulong) != 0) {
        goto do_sigsegv;
    }
    /* FIXME this does not match how the kernel handles the FPU in
     * its sparc64_set_context implementation. In particular the FPU
     * is only restored if fenab is non-zero in:
     *   __get_user(fenab, &(ucp->tuc_mcontext.mc_fpregs.mcfpu_enab));
     */
    __get_user(env->fprs, &(ucp->tuc_mcontext.mc_fpregs.mcfpu_fprs));
    {
        uint32_t *src = ucp->tuc_mcontext.mc_fpregs.mcfpu_fregs.sregs;
        for (i = 0; i < 64; i++, src++) {
            if (i & 1) {
                __get_user(env->fpr[i/2].l.lower, src);
            } else {
                __get_user(env->fpr[i/2].l.upper, src);
            }
        }
    }
    __get_user(env->fsr,
               &(ucp->tuc_mcontext.mc_fpregs.mcfpu_fsr));
    __get_user(env->gsr,
               &(ucp->tuc_mcontext.mc_fpregs.mcfpu_gsr));
    unlock_user_struct(ucp, ucp_addr, 0);
    return;
do_sigsegv:
    unlock_user_struct(ucp, ucp_addr, 0);
    force_sig(TARGET_SIGSEGV);
}

void sparc64_get_context(CPUSPARCState *env)
{
    abi_ulong ucp_addr;
    struct target_ucontext *ucp;
    target_mc_gregset_t *grp;
    target_mcontext_t *mcp;
    abi_ulong fp, i7, w_addr;
    int err;
    unsigned int i;
    target_sigset_t target_set;
    sigset_t set;

    ucp_addr = env->regwptr[UREG_I0];
    if (!lock_user_struct(VERIFY_WRITE, ucp, ucp_addr, 0)) {
        goto do_sigsegv;
    }
    
    mcp = &ucp->tuc_mcontext;
    grp = &mcp->mc_gregs;

    /* Skip over the trap instruction, first. */
    env->pc = env->npc;
    env->npc += 4;

    /* If we're only reading the signal mask then do_sigprocmask()
     * is guaranteed not to fail, which is important because we don't
     * have any way to signal a failure or restart this operation since
     * this is not a normal syscall.
     */
    err = do_sigprocmask(0, NULL, &set);
    assert(err == 0);
    host_to_target_sigset_internal(&target_set, &set);
    if (TARGET_NSIG_WORDS == 1) {
        __put_user(target_set.sig[0],
                   (abi_ulong *)&ucp->tuc_sigmask);
    } else {
        abi_ulong *src, *dst;
        src = target_set.sig;
        dst = ucp->tuc_sigmask.sig;
        for (i = 0; i < TARGET_NSIG_WORDS; i++, dst++, src++) {
            __put_user(*src, dst);
        }
        if (err)
            goto do_sigsegv;
    }

    /* XXX: tstate must be saved properly */
    //    __put_user(env->tstate, &((*grp)[SPARC_MC_TSTATE]));
    __put_user(env->pc, &((*grp)[SPARC_MC_PC]));
    __put_user(env->npc, &((*grp)[SPARC_MC_NPC]));
    __put_user(env->y, &((*grp)[SPARC_MC_Y]));
    __put_user(env->gregs[1], &((*grp)[SPARC_MC_G1]));
    __put_user(env->gregs[2], &((*grp)[SPARC_MC_G2]));
    __put_user(env->gregs[3], &((*grp)[SPARC_MC_G3]));
    __put_user(env->gregs[4], &((*grp)[SPARC_MC_G4]));
    __put_user(env->gregs[5], &((*grp)[SPARC_MC_G5]));
    __put_user(env->gregs[6], &((*grp)[SPARC_MC_G6]));
    __put_user(env->gregs[7], &((*grp)[SPARC_MC_G7]));
    __put_user(env->regwptr[UREG_I0], &((*grp)[SPARC_MC_O0]));
    __put_user(env->regwptr[UREG_I1], &((*grp)[SPARC_MC_O1]));
    __put_user(env->regwptr[UREG_I2], &((*grp)[SPARC_MC_O2]));
    __put_user(env->regwptr[UREG_I3], &((*grp)[SPARC_MC_O3]));
    __put_user(env->regwptr[UREG_I4], &((*grp)[SPARC_MC_O4]));
    __put_user(env->regwptr[UREG_I5], &((*grp)[SPARC_MC_O5]));
    __put_user(env->regwptr[UREG_I6], &((*grp)[SPARC_MC_O6]));
    __put_user(env->regwptr[UREG_I7], &((*grp)[SPARC_MC_O7]));

    w_addr = TARGET_STACK_BIAS+env->regwptr[UREG_I6];
    fp = i7 = 0;
    if (get_user(fp, w_addr + offsetof(struct target_reg_window, ins[6]),
                 abi_ulong) != 0) {
        goto do_sigsegv;
    }
    if (get_user(i7, w_addr + offsetof(struct target_reg_window, ins[7]),
                 abi_ulong) != 0) {
        goto do_sigsegv;
    }
    __put_user(fp, &(mcp->mc_fp));
    __put_user(i7, &(mcp->mc_i7));

    {
        uint32_t *dst = ucp->tuc_mcontext.mc_fpregs.mcfpu_fregs.sregs;
        for (i = 0; i < 64; i++, dst++) {
            if (i & 1) {
                __put_user(env->fpr[i/2].l.lower, dst);
            } else {
                __put_user(env->fpr[i/2].l.upper, dst);
            }
        }
    }
    __put_user(env->fsr, &(mcp->mc_fpregs.mcfpu_fsr));
    __put_user(env->gsr, &(mcp->mc_fpregs.mcfpu_gsr));
    __put_user(env->fprs, &(mcp->mc_fpregs.mcfpu_fprs));

    if (err)
        goto do_sigsegv;
    unlock_user_struct(ucp, ucp_addr, 1);
    return;
do_sigsegv:
    unlock_user_struct(ucp, ucp_addr, 1);
    force_sig(TARGET_SIGSEGV);
}
#endif
#elif defined(TARGET_MIPS) || defined(TARGET_MIPS64)

# if defined(TARGET_ABI_MIPSO32)
struct target_sigcontext {
    uint32_t   sc_regmask;     /* Unused */
    uint32_t   sc_status;
    uint64_t   sc_pc;
    uint64_t   sc_regs[32];
    uint64_t   sc_fpregs[32];
    uint32_t   sc_ownedfp;     /* Unused */
    uint32_t   sc_fpc_csr;
    uint32_t   sc_fpc_eir;     /* Unused */
    uint32_t   sc_used_math;
    uint32_t   sc_dsp;         /* dsp status, was sc_ssflags */
    uint32_t   pad0;
    uint64_t   sc_mdhi;
    uint64_t   sc_mdlo;
    target_ulong   sc_hi1;         /* Was sc_cause */
    target_ulong   sc_lo1;         /* Was sc_badvaddr */
    target_ulong   sc_hi2;         /* Was sc_sigset[4] */
    target_ulong   sc_lo2;
    target_ulong   sc_hi3;
    target_ulong   sc_lo3;
};
# else /* N32 || N64 */
struct target_sigcontext {
    uint64_t sc_regs[32];
    uint64_t sc_fpregs[32];
    uint64_t sc_mdhi;
    uint64_t sc_hi1;
    uint64_t sc_hi2;
    uint64_t sc_hi3;
    uint64_t sc_mdlo;
    uint64_t sc_lo1;
    uint64_t sc_lo2;
    uint64_t sc_lo3;
    uint64_t sc_pc;
    uint32_t sc_fpc_csr;
    uint32_t sc_used_math;
    uint32_t sc_dsp;
    uint32_t sc_reserved;
};
# endif /* O32 */

struct sigframe {
    uint32_t sf_ass[4];			/* argument save space for o32 */
    uint32_t sf_code[2];			/* signal trampoline */
    struct target_sigcontext sf_sc;
    target_sigset_t sf_mask;
};

struct target_ucontext {
    target_ulong tuc_flags;
    target_ulong tuc_link;
    target_stack_t tuc_stack;
    target_ulong pad0;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

struct target_rt_sigframe {
    uint32_t rs_ass[4];               /* argument save space for o32 */
    uint32_t rs_code[2];              /* signal trampoline */
    struct target_siginfo rs_info;
    struct target_ucontext rs_uc;
};

/* Install trampoline to jump back from signal handler */
static inline int install_sigtramp(unsigned int *tramp,   unsigned int syscall)
{
    int err = 0;

    /*
     * Set up the return code ...
     *
     *         li      v0, __NR__foo_sigreturn
     *         syscall
     */

    __put_user(0x24020000 + syscall, tramp + 0);
    __put_user(0x0000000c          , tramp + 1);
    return err;
}

static inline void setup_sigcontext(CPUMIPSState *regs,
                                    struct target_sigcontext *sc)
{
    int i;

    __put_user(exception_resume_pc(regs), &sc->sc_pc);
    regs->hflags &= ~MIPS_HFLAG_BMASK;

    __put_user(0, &sc->sc_regs[0]);
    for (i = 1; i < 32; ++i) {
        __put_user(regs->active_tc.gpr[i], &sc->sc_regs[i]);
    }

    __put_user(regs->active_tc.HI[0], &sc->sc_mdhi);
    __put_user(regs->active_tc.LO[0], &sc->sc_mdlo);

    /* Rather than checking for dsp existence, always copy.  The storage
       would just be garbage otherwise.  */
    __put_user(regs->active_tc.HI[1], &sc->sc_hi1);
    __put_user(regs->active_tc.HI[2], &sc->sc_hi2);
    __put_user(regs->active_tc.HI[3], &sc->sc_hi3);
    __put_user(regs->active_tc.LO[1], &sc->sc_lo1);
    __put_user(regs->active_tc.LO[2], &sc->sc_lo2);
    __put_user(regs->active_tc.LO[3], &sc->sc_lo3);
    {
        uint32_t dsp = cpu_rddsp(0x3ff, regs);
        __put_user(dsp, &sc->sc_dsp);
    }

    __put_user(1, &sc->sc_used_math);

    for (i = 0; i < 32; ++i) {
        __put_user(regs->active_fpu.fpr[i].d, &sc->sc_fpregs[i]);
    }
}

static inline void
restore_sigcontext(CPUMIPSState *regs, struct target_sigcontext *sc)
{
    int i;

    __get_user(regs->CP0_EPC, &sc->sc_pc);

    __get_user(regs->active_tc.HI[0], &sc->sc_mdhi);
    __get_user(regs->active_tc.LO[0], &sc->sc_mdlo);

    for (i = 1; i < 32; ++i) {
        __get_user(regs->active_tc.gpr[i], &sc->sc_regs[i]);
    }

    __get_user(regs->active_tc.HI[1], &sc->sc_hi1);
    __get_user(regs->active_tc.HI[2], &sc->sc_hi2);
    __get_user(regs->active_tc.HI[3], &sc->sc_hi3);
    __get_user(regs->active_tc.LO[1], &sc->sc_lo1);
    __get_user(regs->active_tc.LO[2], &sc->sc_lo2);
    __get_user(regs->active_tc.LO[3], &sc->sc_lo3);
    {
        uint32_t dsp;
        __get_user(dsp, &sc->sc_dsp);
        cpu_wrdsp(dsp, 0x3ff, regs);
    }

    for (i = 0; i < 32; ++i) {
        __get_user(regs->active_fpu.fpr[i].d, &sc->sc_fpregs[i]);
    }
}

/*
 * Determine which stack to use..
 */
static inline abi_ulong
get_sigframe(struct target_sigaction *ka, CPUMIPSState *regs, size_t frame_size)
{
    unsigned long sp;

    /* Default to using normal stack */
    sp = regs->active_tc.gpr[29];

    /*
     * FPU emulator may have its own trampoline active just
     * above the user stack, 16-bytes before the next lowest
     * 16 byte boundary.  Try to avoid trashing it.
     */
    sp -= 32;

    /* This is the X/Open sanctioned signal stack switching.  */
    if ((ka->sa_flags & TARGET_SA_ONSTACK) && (sas_ss_flags (sp) == 0)) {
        sp = target_sigaltstack_used.ss_sp + target_sigaltstack_used.ss_size;
    }

    return (sp - frame_size) & ~7;
}

static void mips_set_hflags_isa_mode_from_pc(CPUMIPSState *env)
{
    if (env->insn_flags & (ASE_MIPS16 | ASE_MICROMIPS)) {
        env->hflags &= ~MIPS_HFLAG_M16;
        env->hflags |= (env->active_tc.PC & 1) << MIPS_HFLAG_M16_SHIFT;
        env->active_tc.PC &= ~(target_ulong) 1;
    }
}

# if defined(TARGET_ABI_MIPSO32)
/* compare linux/arch/mips/kernel/signal.c:setup_frame() */
static void setup_frame(int sig, struct target_sigaction * ka,
                        target_sigset_t *set, CPUMIPSState *regs)
{
    struct sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = get_sigframe(ka, regs, sizeof(*frame));
    trace_user_setup_frame(regs, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    install_sigtramp(frame->sf_code, TARGET_NR_sigreturn);

    setup_sigcontext(regs, &frame->sf_sc);

    for(i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->sf_mask.sig[i]);
    }

    /*
    * Arguments to signal handler:
    *
    *   a0 = signal number
    *   a1 = 0 (should be cause)
    *   a2 = pointer to struct sigcontext
    *
    * $25 and PC point to the signal handler, $29 points to the
    * struct sigframe.
    */
    regs->active_tc.gpr[ 4] = sig;
    regs->active_tc.gpr[ 5] = 0;
    regs->active_tc.gpr[ 6] = frame_addr + offsetof(struct sigframe, sf_sc);
    regs->active_tc.gpr[29] = frame_addr;
    regs->active_tc.gpr[31] = frame_addr + offsetof(struct sigframe, sf_code);
    /* The original kernel code sets CP0_EPC to the handler
    * since it returns to userland using eret
    * we cannot do this here, and we must set PC directly */
    regs->active_tc.PC = regs->active_tc.gpr[25] = ka->_sa_handler;
    mips_set_hflags_isa_mode_from_pc(regs);
    unlock_user_struct(frame, frame_addr, 1);
    return;

give_sigsegv:
    force_sigsegv(sig);
}

long do_sigreturn(CPUMIPSState *regs)
{
    struct sigframe *frame;
    abi_ulong frame_addr;
    sigset_t blocked;
    target_sigset_t target_set;
    int i;

    frame_addr = regs->active_tc.gpr[29];
    trace_user_do_sigreturn(regs, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1))
        goto badframe;

    for(i = 0; i < TARGET_NSIG_WORDS; i++) {
        __get_user(target_set.sig[i], &frame->sf_mask.sig[i]);
    }

    target_to_host_sigset_internal(&blocked, &target_set);
    set_sigmask(&blocked);

    restore_sigcontext(regs, &frame->sf_sc);

#if 0
    /*
     * Don't let your children do this ...
     */
    __asm__ __volatile__(
   	"move\t$29, %0\n\t"
   	"j\tsyscall_exit"
   	:/* no outputs */
   	:"r" (&regs));
    /* Unreached */
#endif

    regs->active_tc.PC = regs->CP0_EPC;
    mips_set_hflags_isa_mode_from_pc(regs);
    /* I am not sure this is right, but it seems to work
    * maybe a problem with nested signals ? */
    regs->CP0_EPC = 0;
    return -TARGET_QEMU_ESIGRETURN;

badframe:
    force_sig(TARGET_SIGSEGV);
    return -TARGET_QEMU_ESIGRETURN;
}
# endif /* O32 */

static void setup_rt_frame(int sig, struct target_sigaction *ka,
                           target_siginfo_t *info,
                           target_sigset_t *set, CPUMIPSState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    install_sigtramp(frame->rs_code, TARGET_NR_rt_sigreturn);

    tswap_siginfo(&frame->rs_info, info);

    __put_user(0, &frame->rs_uc.tuc_flags);
    __put_user(0, &frame->rs_uc.tuc_link);
    __put_user(target_sigaltstack_used.ss_sp, &frame->rs_uc.tuc_stack.ss_sp);
    __put_user(target_sigaltstack_used.ss_size, &frame->rs_uc.tuc_stack.ss_size);
    __put_user(sas_ss_flags(get_sp_from_cpustate(env)),
               &frame->rs_uc.tuc_stack.ss_flags);

    setup_sigcontext(env, &frame->rs_uc.tuc_mcontext);

    for(i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->rs_uc.tuc_sigmask.sig[i]);
    }

    /*
    * Arguments to signal handler:
    *
    *   a0 = signal number
    *   a1 = pointer to siginfo_t
    *   a2 = pointer to ucontext_t
    *
    * $25 and PC point to the signal handler, $29 points to the
    * struct sigframe.
    */
    env->active_tc.gpr[ 4] = sig;
    env->active_tc.gpr[ 5] = frame_addr
                             + offsetof(struct target_rt_sigframe, rs_info);
    env->active_tc.gpr[ 6] = frame_addr
                             + offsetof(struct target_rt_sigframe, rs_uc);
    env->active_tc.gpr[29] = frame_addr;
    env->active_tc.gpr[31] = frame_addr
                             + offsetof(struct target_rt_sigframe, rs_code);
    /* The original kernel code sets CP0_EPC to the handler
    * since it returns to userland using eret
    * we cannot do this here, and we must set PC directly */
    env->active_tc.PC = env->active_tc.gpr[25] = ka->_sa_handler;
    mips_set_hflags_isa_mode_from_pc(env);
    unlock_user_struct(frame, frame_addr, 1);
    return;

give_sigsegv:
    unlock_user_struct(frame, frame_addr, 1);
    force_sigsegv(sig);
}

long do_rt_sigreturn(CPUMIPSState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;
    sigset_t blocked;

    frame_addr = env->active_tc.gpr[29];
    trace_user_do_rt_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }

    target_to_host_sigset(&blocked, &frame->rs_uc.tuc_sigmask);
    set_sigmask(&blocked);

    restore_sigcontext(env, &frame->rs_uc.tuc_mcontext);

    if (do_sigaltstack(frame_addr +
                       offsetof(struct target_rt_sigframe, rs_uc.tuc_stack),
                       0, get_sp_from_cpustate(env)) == -EFAULT)
        goto badframe;

    env->active_tc.PC = env->CP0_EPC;
    mips_set_hflags_isa_mode_from_pc(env);
    /* I am not sure this is right, but it seems to work
    * maybe a problem with nested signals ? */
    env->CP0_EPC = 0;
    return -TARGET_QEMU_ESIGRETURN;

badframe:
    force_sig(TARGET_SIGSEGV);
    return -TARGET_QEMU_ESIGRETURN;
}

#elif defined(TARGET_PPC)

/* Size of dummy stack frame allocated when calling signal handler.
   See arch/powerpc/include/asm/ptrace.h.  */
#if defined(TARGET_PPC64)
#define SIGNAL_FRAMESIZE 128
#else
#define SIGNAL_FRAMESIZE 64
#endif

/* See arch/powerpc/include/asm/ucontext.h.  Only used for 32-bit PPC;
   on 64-bit PPC, sigcontext and mcontext are one and the same.  */
struct target_mcontext {
    target_ulong mc_gregs[48];
    /* Includes fpscr.  */
    uint64_t mc_fregs[33];
#if defined(TARGET_PPC64)
    /* Pointer to the vector regs */
    target_ulong v_regs;
#else
    target_ulong mc_pad[2];
#endif
    /* We need to handle Altivec and SPE at the same time, which no
       kernel needs to do.  Fortunately, the kernel defines this bit to
       be Altivec-register-large all the time, rather than trying to
       twiddle it based on the specific platform.  */
    union {
        /* SPE vector registers.  One extra for SPEFSCR.  */
        uint32_t spe[33];
        /* Altivec vector registers.  The packing of VSCR and VRSAVE
           varies depending on whether we're PPC64 or not: PPC64 splits
           them apart; PPC32 stuffs them together.
           We also need to account for the VSX registers on PPC64
        */
#if defined(TARGET_PPC64)
#define QEMU_NVRREG (34 + 16)
        /* On ppc64, this mcontext structure is naturally *unaligned*,
         * or rather it is aligned on a 8 bytes boundary but not on
         * a 16 bytes one. This pad fixes it up. This is also why the
         * vector regs are referenced by the v_regs pointer above so
         * any amount of padding can be added here
         */
        target_ulong pad;
#else
        /* On ppc32, we are already aligned to 16 bytes */
#define QEMU_NVRREG 33
#endif
        /* We cannot use ppc_avr_t here as we do *not* want the implied
         * 16-bytes alignment that would result from it. This would have
         * the effect of making the whole struct target_mcontext aligned
         * which breaks the layout of struct target_ucontext on ppc64.
         */
        uint64_t altivec[QEMU_NVRREG][2];
#undef QEMU_NVRREG
    } mc_vregs;
};

/* See arch/powerpc/include/asm/sigcontext.h.  */
struct target_sigcontext {
    target_ulong _unused[4];
    int32_t signal;
#if defined(TARGET_PPC64)
    int32_t pad0;
#endif
    target_ulong handler;
    target_ulong oldmask;
    target_ulong regs;      /* struct pt_regs __user * */
#if defined(TARGET_PPC64)
    struct target_mcontext mcontext;
#endif
};

/* Indices for target_mcontext.mc_gregs, below.
   See arch/powerpc/include/asm/ptrace.h for details.  */
enum {
    TARGET_PT_R0 = 0,
    TARGET_PT_R1 = 1,
    TARGET_PT_R2 = 2,
    TARGET_PT_R3 = 3,
    TARGET_PT_R4 = 4,
    TARGET_PT_R5 = 5,
    TARGET_PT_R6 = 6,
    TARGET_PT_R7 = 7,
    TARGET_PT_R8 = 8,
    TARGET_PT_R9 = 9,
    TARGET_PT_R10 = 10,
    TARGET_PT_R11 = 11,
    TARGET_PT_R12 = 12,
    TARGET_PT_R13 = 13,
    TARGET_PT_R14 = 14,
    TARGET_PT_R15 = 15,
    TARGET_PT_R16 = 16,
    TARGET_PT_R17 = 17,
    TARGET_PT_R18 = 18,
    TARGET_PT_R19 = 19,
    TARGET_PT_R20 = 20,
    TARGET_PT_R21 = 21,
    TARGET_PT_R22 = 22,
    TARGET_PT_R23 = 23,
    TARGET_PT_R24 = 24,
    TARGET_PT_R25 = 25,
    TARGET_PT_R26 = 26,
    TARGET_PT_R27 = 27,
    TARGET_PT_R28 = 28,
    TARGET_PT_R29 = 29,
    TARGET_PT_R30 = 30,
    TARGET_PT_R31 = 31,
    TARGET_PT_NIP = 32,
    TARGET_PT_MSR = 33,
    TARGET_PT_ORIG_R3 = 34,
    TARGET_PT_CTR = 35,
    TARGET_PT_LNK = 36,
    TARGET_PT_XER = 37,
    TARGET_PT_CCR = 38,
    /* Yes, there are two registers with #39.  One is 64-bit only.  */
    TARGET_PT_MQ = 39,
    TARGET_PT_SOFTE = 39,
    TARGET_PT_TRAP = 40,
    TARGET_PT_DAR = 41,
    TARGET_PT_DSISR = 42,
    TARGET_PT_RESULT = 43,
    TARGET_PT_REGS_COUNT = 44
};


struct target_ucontext {
    target_ulong tuc_flags;
    target_ulong tuc_link;    /* ucontext_t __user * */
    struct target_sigaltstack tuc_stack;
#if !defined(TARGET_PPC64)
    int32_t tuc_pad[7];
    target_ulong tuc_regs;    /* struct mcontext __user *
                                points to uc_mcontext field */
#endif
    target_sigset_t tuc_sigmask;
#if defined(TARGET_PPC64)
    target_sigset_t unused[15]; /* Allow for uc_sigmask growth */
    struct target_sigcontext tuc_sigcontext;
#else
    int32_t tuc_maskext[30];
    int32_t tuc_pad2[3];
    struct target_mcontext tuc_mcontext;
#endif
};

/* See arch/powerpc/kernel/signal_32.c.  */
struct target_sigframe {
    struct target_sigcontext sctx;
    struct target_mcontext mctx;
    int32_t abigap[56];
};

#if defined(TARGET_PPC64)

#define TARGET_TRAMP_SIZE 6

struct target_rt_sigframe {
    /* sys_rt_sigreturn requires the ucontext be the first field */
    struct target_ucontext uc;
    target_ulong  _unused[2];
    uint32_t trampoline[TARGET_TRAMP_SIZE];
    target_ulong pinfo; /* struct siginfo __user * */
    target_ulong puc; /* void __user * */
    struct target_siginfo info;
    /* 64 bit ABI allows for 288 bytes below sp before decrementing it. */
    char abigap[288];
} __attribute__((aligned(16)));

#else

struct target_rt_sigframe {
    struct target_siginfo info;
    struct target_ucontext uc;
    int32_t abigap[56];
};

#endif

#if defined(TARGET_PPC64)

struct target_func_ptr {
    target_ulong entry;
    target_ulong toc;
};

#endif

/* We use the mc_pad field for the signal return trampoline.  */
#define tramp mc_pad

/* See arch/powerpc/kernel/signal.c.  */
static target_ulong get_sigframe(struct target_sigaction *ka,
                                 CPUPPCState *env,
                                 int frame_size)
{
    target_ulong oldsp;

    oldsp = env->gpr[1];

    if ((ka->sa_flags & TARGET_SA_ONSTACK) &&
            (sas_ss_flags(oldsp) == 0)) {
        oldsp = (target_sigaltstack_used.ss_sp
                 + target_sigaltstack_used.ss_size);
    }

    return (oldsp - frame_size) & ~0xFUL;
}

#if ((defined(TARGET_WORDS_BIGENDIAN) && defined(HOST_WORDS_BIGENDIAN)) || \
     (!defined(HOST_WORDS_BIGENDIAN) && !defined(TARGET_WORDS_BIGENDIAN)))
#define PPC_VEC_HI      0
#define PPC_VEC_LO      1
#else
#define PPC_VEC_HI      1
#define PPC_VEC_LO      0
#endif


static void save_user_regs(CPUPPCState *env, struct target_mcontext *frame)
{
    target_ulong msr = env->msr;
    int i;
    target_ulong ccr = 0;

    /* In general, the kernel attempts to be intelligent about what it
       needs to save for Altivec/FP/SPE registers.  We don't care that
       much, so we just go ahead and save everything.  */

    /* Save general registers.  */
    for (i = 0; i < ARRAY_SIZE(env->gpr); i++) {
        __put_user(env->gpr[i], &frame->mc_gregs[i]);
    }
    __put_user(env->nip, &frame->mc_gregs[TARGET_PT_NIP]);
    __put_user(env->ctr, &frame->mc_gregs[TARGET_PT_CTR]);
    __put_user(env->lr, &frame->mc_gregs[TARGET_PT_LNK]);
    __put_user(env->xer, &frame->mc_gregs[TARGET_PT_XER]);

    for (i = 0; i < ARRAY_SIZE(env->crf); i++) {
        ccr |= env->crf[i] << (32 - ((i + 1) * 4));
    }
    __put_user(ccr, &frame->mc_gregs[TARGET_PT_CCR]);

    /* Save Altivec registers if necessary.  */
    if (env->insns_flags & PPC_ALTIVEC) {
        uint32_t *vrsave;
        for (i = 0; i < ARRAY_SIZE(env->avr); i++) {
            ppc_avr_t *avr = &env->avr[i];
            ppc_avr_t *vreg = (ppc_avr_t *)&frame->mc_vregs.altivec[i];

            __put_user(avr->u64[PPC_VEC_HI], &vreg->u64[0]);
            __put_user(avr->u64[PPC_VEC_LO], &vreg->u64[1]);
        }
        /* Set MSR_VR in the saved MSR value to indicate that
           frame->mc_vregs contains valid data.  */
        msr |= MSR_VR;
#if defined(TARGET_PPC64)
        vrsave = (uint32_t *)&frame->mc_vregs.altivec[33];
        /* 64-bit needs to put a pointer to the vectors in the frame */
        __put_user(h2g(frame->mc_vregs.altivec), &frame->v_regs);
#else
        vrsave = (uint32_t *)&frame->mc_vregs.altivec[32];
#endif
        __put_user((uint32_t)env->spr[SPR_VRSAVE], vrsave);
    }

    /* Save VSX second halves */
    if (env->insns_flags2 & PPC2_VSX) {
        uint64_t *vsregs = (uint64_t *)&frame->mc_vregs.altivec[34];
        for (i = 0; i < ARRAY_SIZE(env->vsr); i++) {
            __put_user(env->vsr[i], &vsregs[i]);
        }
    }

    /* Save floating point registers.  */
    if (env->insns_flags & PPC_FLOAT) {
        for (i = 0; i < ARRAY_SIZE(env->fpr); i++) {
            __put_user(env->fpr[i], &frame->mc_fregs[i]);
        }
        __put_user((uint64_t) env->fpscr, &frame->mc_fregs[32]);
    }

    /* Save SPE registers.  The kernel only saves the high half.  */
    if (env->insns_flags & PPC_SPE) {
#if defined(TARGET_PPC64)
        for (i = 0; i < ARRAY_SIZE(env->gpr); i++) {
            __put_user(env->gpr[i] >> 32, &frame->mc_vregs.spe[i]);
        }
#else
        for (i = 0; i < ARRAY_SIZE(env->gprh); i++) {
            __put_user(env->gprh[i], &frame->mc_vregs.spe[i]);
        }
#endif
        /* Set MSR_SPE in the saved MSR value to indicate that
           frame->mc_vregs contains valid data.  */
        msr |= MSR_SPE;
        __put_user(env->spe_fscr, &frame->mc_vregs.spe[32]);
    }

    /* Store MSR.  */
    __put_user(msr, &frame->mc_gregs[TARGET_PT_MSR]);
}

static void encode_trampoline(int sigret, uint32_t *tramp)
{
    /* Set up the sigreturn trampoline: li r0,sigret; sc.  */
    if (sigret) {
        __put_user(0x38000000 | sigret, &tramp[0]);
        __put_user(0x44000002, &tramp[1]);
    }
}

static void restore_user_regs(CPUPPCState *env,
                              struct target_mcontext *frame, int sig)
{
    target_ulong save_r2 = 0;
    target_ulong msr;
    target_ulong ccr;

    int i;

    if (!sig) {
        save_r2 = env->gpr[2];
    }

    /* Restore general registers.  */
    for (i = 0; i < ARRAY_SIZE(env->gpr); i++) {
        __get_user(env->gpr[i], &frame->mc_gregs[i]);
    }
    __get_user(env->nip, &frame->mc_gregs[TARGET_PT_NIP]);
    __get_user(env->ctr, &frame->mc_gregs[TARGET_PT_CTR]);
    __get_user(env->lr, &frame->mc_gregs[TARGET_PT_LNK]);
    __get_user(env->xer, &frame->mc_gregs[TARGET_PT_XER]);
    __get_user(ccr, &frame->mc_gregs[TARGET_PT_CCR]);

    for (i = 0; i < ARRAY_SIZE(env->crf); i++) {
        env->crf[i] = (ccr >> (32 - ((i + 1) * 4))) & 0xf;
    }

    if (!sig) {
        env->gpr[2] = save_r2;
    }
    /* Restore MSR.  */
    __get_user(msr, &frame->mc_gregs[TARGET_PT_MSR]);

    /* If doing signal return, restore the previous little-endian mode.  */
    if (sig)
        env->msr = (env->msr & ~(1ull << MSR_LE)) | (msr & (1ull << MSR_LE));

    /* Restore Altivec registers if necessary.  */
    if (env->insns_flags & PPC_ALTIVEC) {
        ppc_avr_t *v_regs;
        uint32_t *vrsave;
#if defined(TARGET_PPC64)
        uint64_t v_addr;
        /* 64-bit needs to recover the pointer to the vectors from the frame */
        __get_user(v_addr, &frame->v_regs);
        v_regs = g2h(v_addr);
#else
        v_regs = (ppc_avr_t *)frame->mc_vregs.altivec;
#endif
        for (i = 0; i < ARRAY_SIZE(env->avr); i++) {
            ppc_avr_t *avr = &env->avr[i];
            ppc_avr_t *vreg = &v_regs[i];

            __get_user(avr->u64[PPC_VEC_HI], &vreg->u64[0]);
            __get_user(avr->u64[PPC_VEC_LO], &vreg->u64[1]);
        }
        /* Set MSR_VEC in the saved MSR value to indicate that
           frame->mc_vregs contains valid data.  */
#if defined(TARGET_PPC64)
        vrsave = (uint32_t *)&v_regs[33];
#else
        vrsave = (uint32_t *)&v_regs[32];
#endif
        __get_user(env->spr[SPR_VRSAVE], vrsave);
    }

    /* Restore VSX second halves */
    if (env->insns_flags2 & PPC2_VSX) {
        uint64_t *vsregs = (uint64_t *)&frame->mc_vregs.altivec[34];
        for (i = 0; i < ARRAY_SIZE(env->vsr); i++) {
            __get_user(env->vsr[i], &vsregs[i]);
        }
    }

    /* Restore floating point registers.  */
    if (env->insns_flags & PPC_FLOAT) {
        uint64_t fpscr;
        for (i = 0; i < ARRAY_SIZE(env->fpr); i++) {
            __get_user(env->fpr[i], &frame->mc_fregs[i]);
        }
        __get_user(fpscr, &frame->mc_fregs[32]);
        env->fpscr = (uint32_t) fpscr;
    }

    /* Save SPE registers.  The kernel only saves the high half.  */
    if (env->insns_flags & PPC_SPE) {
#if defined(TARGET_PPC64)
        for (i = 0; i < ARRAY_SIZE(env->gpr); i++) {
            uint32_t hi;

            __get_user(hi, &frame->mc_vregs.spe[i]);
            env->gpr[i] = ((uint64_t)hi << 32) | ((uint32_t) env->gpr[i]);
        }
#else
        for (i = 0; i < ARRAY_SIZE(env->gprh); i++) {
            __get_user(env->gprh[i], &frame->mc_vregs.spe[i]);
        }
#endif
        __get_user(env->spe_fscr, &frame->mc_vregs.spe[32]);
    }
}

#if !defined(TARGET_PPC64)
static void setup_frame(int sig, struct target_sigaction *ka,
                        target_sigset_t *set, CPUPPCState *env)
{
    struct target_sigframe *frame;
    struct target_sigcontext *sc;
    target_ulong frame_addr, newsp;
    int err = 0;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_frame(env, frame_addr);
    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 1))
        goto sigsegv;
    sc = &frame->sctx;

    __put_user(ka->_sa_handler, &sc->handler);
    __put_user(set->sig[0], &sc->oldmask);
    __put_user(set->sig[1], &sc->_unused[3]);
    __put_user(h2g(&frame->mctx), &sc->regs);
    __put_user(sig, &sc->signal);

    /* Save user regs.  */
    save_user_regs(env, &frame->mctx);

    /* Construct the trampoline code on the stack. */
    encode_trampoline(TARGET_NR_sigreturn, (uint32_t *)&frame->mctx.tramp);

    /* The kernel checks for the presence of a VDSO here.  We don't
       emulate a vdso, so use a sigreturn system call.  */
    env->lr = (target_ulong) h2g(frame->mctx.tramp);

    /* Turn off all fp exceptions.  */
    env->fpscr = 0;

    /* Create a stack frame for the caller of the handler.  */
    newsp = frame_addr - SIGNAL_FRAMESIZE;
    err |= put_user(env->gpr[1], newsp, target_ulong);

    if (err)
        goto sigsegv;

    /* Set up registers for signal handler.  */
    env->gpr[1] = newsp;
    env->gpr[3] = sig;
    env->gpr[4] = frame_addr + offsetof(struct target_sigframe, sctx);

    env->nip = (target_ulong) ka->_sa_handler;

    /* Signal handlers are entered in big-endian mode.  */
    env->msr &= ~(1ull << MSR_LE);

    unlock_user_struct(frame, frame_addr, 1);
    return;

sigsegv:
    unlock_user_struct(frame, frame_addr, 1);
    force_sigsegv(sig);
}
#endif /* !defined(TARGET_PPC64) */

static void setup_rt_frame(int sig, struct target_sigaction *ka,
                           target_siginfo_t *info,
                           target_sigset_t *set, CPUPPCState *env)
{
    struct target_rt_sigframe *rt_sf;
    uint32_t *trampptr = 0;
    struct target_mcontext *mctx = 0;
    target_ulong rt_sf_addr, newsp = 0;
    int i, err = 0;
#if defined(TARGET_PPC64)
    struct target_sigcontext *sc = 0;
    struct image_info *image = ((TaskState *)thread_cpu->opaque)->info;
#endif

    rt_sf_addr = get_sigframe(ka, env, sizeof(*rt_sf));
    if (!lock_user_struct(VERIFY_WRITE, rt_sf, rt_sf_addr, 1))
        goto sigsegv;

    tswap_siginfo(&rt_sf->info, info);

    __put_user(0, &rt_sf->uc.tuc_flags);
    __put_user(0, &rt_sf->uc.tuc_link);
    __put_user((target_ulong)target_sigaltstack_used.ss_sp,
               &rt_sf->uc.tuc_stack.ss_sp);
    __put_user(sas_ss_flags(env->gpr[1]),
               &rt_sf->uc.tuc_stack.ss_flags);
    __put_user(target_sigaltstack_used.ss_size,
               &rt_sf->uc.tuc_stack.ss_size);
#if !defined(TARGET_PPC64)
    __put_user(h2g (&rt_sf->uc.tuc_mcontext),
               &rt_sf->uc.tuc_regs);
#endif
    for(i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &rt_sf->uc.tuc_sigmask.sig[i]);
    }

#if defined(TARGET_PPC64)
    mctx = &rt_sf->uc.tuc_sigcontext.mcontext;
    trampptr = &rt_sf->trampoline[0];

    sc = &rt_sf->uc.tuc_sigcontext;
    __put_user(h2g(mctx), &sc->regs);
    __put_user(sig, &sc->signal);
#else
    mctx = &rt_sf->uc.tuc_mcontext;
    trampptr = (uint32_t *)&rt_sf->uc.tuc_mcontext.tramp;
#endif

    save_user_regs(env, mctx);
    encode_trampoline(TARGET_NR_rt_sigreturn, trampptr);

    /* The kernel checks for the presence of a VDSO here.  We don't
       emulate a vdso, so use a sigreturn system call.  */
    env->lr = (target_ulong) h2g(trampptr);

    /* Turn off all fp exceptions.  */
    env->fpscr = 0;

    /* Create a stack frame for the caller of the handler.  */
    newsp = rt_sf_addr - (SIGNAL_FRAMESIZE + 16);
    err |= put_user(env->gpr[1], newsp, target_ulong);

    if (err)
        goto sigsegv;

    /* Set up registers for signal handler.  */
    env->gpr[1] = newsp;
    env->gpr[3] = (target_ulong) sig;
    env->gpr[4] = (target_ulong) h2g(&rt_sf->info);
    env->gpr[5] = (target_ulong) h2g(&rt_sf->uc);
    env->gpr[6] = (target_ulong) h2g(rt_sf);

#if defined(TARGET_PPC64)
    if (get_ppc64_abi(image) < 2) {
        /* ELFv1 PPC64 function pointers are pointers to OPD entries. */
        struct target_func_ptr *handler =
            (struct target_func_ptr *)g2h(ka->_sa_handler);
        env->nip = tswapl(handler->entry);
        env->gpr[2] = tswapl(handler->toc);
    } else {
        /* ELFv2 PPC64 function pointers are entry points, but R12
         * must also be set */
        env->nip = tswapl((target_ulong) ka->_sa_handler);
        env->gpr[12] = env->nip;
    }
#else
    env->nip = (target_ulong) ka->_sa_handler;
#endif

    /* Signal handlers are entered in big-endian mode.  */
    env->msr &= ~(1ull << MSR_LE);

    unlock_user_struct(rt_sf, rt_sf_addr, 1);
    return;

sigsegv:
    unlock_user_struct(rt_sf, rt_sf_addr, 1);
    force_sigsegv(sig);

}

#if !defined(TARGET_PPC64)
long do_sigreturn(CPUPPCState *env)
{
    struct target_sigcontext *sc = NULL;
    struct target_mcontext *sr = NULL;
    target_ulong sr_addr = 0, sc_addr;
    sigset_t blocked;
    target_sigset_t set;

    sc_addr = env->gpr[1] + SIGNAL_FRAMESIZE;
    if (!lock_user_struct(VERIFY_READ, sc, sc_addr, 1))
        goto sigsegv;

#if defined(TARGET_PPC64)
    set.sig[0] = sc->oldmask + ((uint64_t)(sc->_unused[3]) << 32);
#else
    __get_user(set.sig[0], &sc->oldmask);
    __get_user(set.sig[1], &sc->_unused[3]);
#endif
    target_to_host_sigset_internal(&blocked, &set);
    set_sigmask(&blocked);

    __get_user(sr_addr, &sc->regs);
    if (!lock_user_struct(VERIFY_READ, sr, sr_addr, 1))
        goto sigsegv;
    restore_user_regs(env, sr, 1);

    unlock_user_struct(sr, sr_addr, 1);
    unlock_user_struct(sc, sc_addr, 1);
    return -TARGET_QEMU_ESIGRETURN;

sigsegv:
    unlock_user_struct(sr, sr_addr, 1);
    unlock_user_struct(sc, sc_addr, 1);
    force_sig(TARGET_SIGSEGV);
    return -TARGET_QEMU_ESIGRETURN;
}
#endif /* !defined(TARGET_PPC64) */

/* See arch/powerpc/kernel/signal_32.c.  */
static int do_setcontext(struct target_ucontext *ucp, CPUPPCState *env, int sig)
{
    struct target_mcontext *mcp;
    target_ulong mcp_addr;
    sigset_t blocked;
    target_sigset_t set;

    if (copy_from_user(&set, h2g(ucp) + offsetof(struct target_ucontext, tuc_sigmask),
                       sizeof (set)))
        return 1;

#if defined(TARGET_PPC64)
    mcp_addr = h2g(ucp) +
        offsetof(struct target_ucontext, tuc_sigcontext.mcontext);
#else
    __get_user(mcp_addr, &ucp->tuc_regs);
#endif

    if (!lock_user_struct(VERIFY_READ, mcp, mcp_addr, 1))
        return 1;

    target_to_host_sigset_internal(&blocked, &set);
    set_sigmask(&blocked);
    restore_user_regs(env, mcp, sig);

    unlock_user_struct(mcp, mcp_addr, 1);
    return 0;
}

long do_rt_sigreturn(CPUPPCState *env)
{
    struct target_rt_sigframe *rt_sf = NULL;
    target_ulong rt_sf_addr;

    rt_sf_addr = env->gpr[1] + SIGNAL_FRAMESIZE + 16;
    if (!lock_user_struct(VERIFY_READ, rt_sf, rt_sf_addr, 1))
        goto sigsegv;

    if (do_setcontext(&rt_sf->uc, env, 1))
        goto sigsegv;

    do_sigaltstack(rt_sf_addr
                   + offsetof(struct target_rt_sigframe, uc.tuc_stack),
                   0, env->gpr[1]);

    unlock_user_struct(rt_sf, rt_sf_addr, 1);
    return -TARGET_QEMU_ESIGRETURN;

sigsegv:
    unlock_user_struct(rt_sf, rt_sf_addr, 1);
    force_sig(TARGET_SIGSEGV);
    return -TARGET_QEMU_ESIGRETURN;
}

#elif defined(TARGET_RISCV)

/* Signal handler invocation must be transparent for the code being
   interrupted. Complete CPU (hart) state is saved on entry and restored
   before returning from the handler. Process sigmask is also saved to block
   signals while the handler is running. The handler gets its own stack,
   which also doubles as storage for the CPU state and sigmask.

   The code below is qemu re-implementation of arch/riscv/kernel/signal.c */

struct target_sigcontext {
    abi_long pc;
    abi_long gpr[31]; /* x0 is not present, so all offsets must be -1 */
    uint64_t fpr[32];
    uint32_t fcsr;
}; /* cf. riscv-linux:arch/riscv/include/uapi/asm/ptrace.h */

struct target_ucontext {
    unsigned long uc_flags;
    struct target_ucontext *uc_link;
    target_stack_t uc_stack;
    struct target_sigcontext uc_mcontext;
    target_sigset_t uc_sigmask;
};

struct target_rt_sigframe {
    uint32_t tramp[2]; /* not in kernel, which uses VDSO instead */
    struct target_siginfo info;
    struct target_ucontext uc;
};

static abi_ulong get_sigframe(struct target_sigaction *ka,
                              CPURISCVState *regs, size_t framesize)
{
    abi_ulong sp = regs->gpr[xSP];
    int onsigstack = on_sig_stack(sp);

    /* redzone */
    /* This is the X/Open sanctioned signal stack switching.  */
    if ((ka->sa_flags & TARGET_SA_ONSTACK) != 0 && !onsigstack) {
        sp = target_sigaltstack_used.ss_sp + target_sigaltstack_used.ss_size;
    }

    sp -= framesize;
    sp &= ~3UL; /* align sp on 4-byte boundary */

    /* If we are on the alternate signal stack and would overflow it, don't.
       Return an always-bogus address instead so we will die with SIGSEGV. */
    if (onsigstack && !likely(on_sig_stack(sp))) {
        return -1L;
    }

    return sp;
}

static void setup_sigcontext(struct target_sigcontext *sc, CPURISCVState *env)
{
    int i;

    __put_user(env->pc, &sc->pc);

    for (i = 1; i < 32; i++) {
        __put_user(env->gpr[i], &sc->gpr[i - 1]);
    }
    for (i = 0; i < 32; i++) {
        __put_user(env->fpr[i], &sc->fpr[i]);
    }

    uint32_t fcsr = csr_read_helper(env, CSR_FCSR); /*riscv_get_fcsr(env);*/
    __put_user(fcsr, &sc->fcsr);
}

static void setup_ucontext(struct target_ucontext *uc,
                           CPURISCVState *env, target_sigset_t *set)
{
    abi_ulong ss_sp = (target_ulong)target_sigaltstack_used.ss_sp;
    abi_ulong ss_flags = sas_ss_flags(env->gpr[xSP]);
    abi_ulong ss_size = target_sigaltstack_used.ss_size;

    __put_user(0,    &(uc->uc_flags));
    __put_user(0,    &(uc->uc_link));

    __put_user(ss_sp,    &(uc->uc_stack.ss_sp));
    __put_user(ss_flags, &(uc->uc_stack.ss_flags));
    __put_user(ss_size,  &(uc->uc_stack.ss_size));

    int i;
    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &(uc->uc_sigmask.sig[i]));
    }

    setup_sigcontext(&uc->uc_mcontext, env);
}

static inline void install_sigtramp(uint32_t *tramp)
{
    __put_user(0x08b00893, tramp + 0);  /* li a7, 139 = __NR_rt_sigreturn */
    __put_user(0x00000073, tramp + 1);  /* ecall */
}

static void setup_rt_frame(int sig, struct target_sigaction *ka,
                           target_siginfo_t *info,
                           target_sigset_t *set, CPURISCVState *env)
{
    abi_ulong frame_addr;
    struct target_rt_sigframe *frame;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);

    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto badframe;
    }

    setup_ucontext(&frame->uc, env, set);
    tswap_siginfo(&frame->info, info);
    install_sigtramp(frame->tramp);

    env->pc = ka->_sa_handler;
    env->gpr[xSP] = frame_addr;
    env->gpr[xA0] = sig;
    env->gpr[xA1] = frame_addr + offsetof(struct target_rt_sigframe, info);
    env->gpr[xA2] = frame_addr + offsetof(struct target_rt_sigframe, uc);
    env->gpr[xRA] = frame_addr + offsetof(struct target_rt_sigframe, tramp);

    return;

badframe:
    unlock_user_struct(frame, frame_addr, 1);
    if (sig == TARGET_SIGSEGV) {
        ka->_sa_handler = TARGET_SIG_DFL;
    }
    force_sig(TARGET_SIGSEGV);
}

static void restore_sigcontext(CPURISCVState *env, struct target_sigcontext *sc)
{
    int i;

    __get_user(env->pc, &sc->pc);

    for (i = 1; i < 32; ++i) {
        __get_user(env->gpr[i], &sc->gpr[i - 1]);
    }
    for (i = 0; i < 32; ++i) {
        __get_user(env->fpr[i], &sc->fpr[i]);
    }

    uint32_t fcsr;
    __get_user(fcsr, &sc->fcsr);
    csr_write_helper(env, fcsr, CSR_FCSR);
}

static void restore_ucontext(CPURISCVState *env, struct target_ucontext *uc)
{
    sigset_t blocked;
    target_sigset_t target_set;
    int i;

    target_sigemptyset(&target_set);
    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __get_user(target_set.sig[i], &(uc->uc_sigmask.sig[i]));
    }

    target_to_host_sigset_internal(&blocked, &target_set);
    set_sigmask(&blocked);

    restore_sigcontext(env, &uc->uc_mcontext);
}

long do_rt_sigreturn(CPURISCVState *env)
{
    struct target_rt_sigframe *frame;
    abi_ulong frame_addr;

    frame_addr = env->gpr[xSP];
    trace_user_do_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }

    restore_ucontext(env, &frame->uc);

    if (do_sigaltstack(frame_addr + offsetof(struct target_rt_sigframe,
            uc.uc_stack), 0, get_sp_from_cpustate(env)) == -EFAULT) {
        goto badframe;
    }

    unlock_user_struct(frame, frame_addr, 0);
    return -TARGET_QEMU_ESIGRETURN;

badframe:
    unlock_user_struct(frame, frame_addr, 0);
    force_sig(TARGET_SIGSEGV);
    return 0;
}

#elif defined(TARGET_HPPA)

struct target_sigcontext {
    abi_ulong sc_flags;
    abi_ulong sc_gr[32];
    uint64_t sc_fr[32];
    abi_ulong sc_iasq[2];
    abi_ulong sc_iaoq[2];
    abi_ulong sc_sar;
};

struct target_ucontext {
    abi_uint tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    abi_uint pad[1];
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

struct target_rt_sigframe {
    abi_uint tramp[9];
    target_siginfo_t info;
    struct target_ucontext uc;
    /* hidden location of upper halves of pa2.0 64-bit gregs */
};

static void setup_sigcontext(struct target_sigcontext *sc, CPUArchState *env)
{
    int flags = 0;
    int i;

    /* ??? if on_sig_stack, flags |= 1 (PARISC_SC_FLAG_ONSTACK).  */

    if (env->iaoq_f < TARGET_PAGE_SIZE) {
        /* In the gateway page, executing a syscall.  */
        flags |= 2; /* PARISC_SC_FLAG_IN_SYSCALL */
        __put_user(env->gr[31], &sc->sc_iaoq[0]);
        __put_user(env->gr[31] + 4, &sc->sc_iaoq[1]);
    } else {
        __put_user(env->iaoq_f, &sc->sc_iaoq[0]);
        __put_user(env->iaoq_b, &sc->sc_iaoq[1]);
    }
    __put_user(0, &sc->sc_iasq[0]);
    __put_user(0, &sc->sc_iasq[1]);
    __put_user(flags, &sc->sc_flags);

    __put_user(cpu_hppa_get_psw(env), &sc->sc_gr[0]);
    for (i = 1; i < 32; ++i) {
        __put_user(env->gr[i], &sc->sc_gr[i]);
    }

    __put_user((uint64_t)env->fr0_shadow << 32, &sc->sc_fr[0]);
    for (i = 1; i < 32; ++i) {
        __put_user(env->fr[i], &sc->sc_fr[i]);
    }

    __put_user(env->cr[CR_SAR], &sc->sc_sar);
}

static void restore_sigcontext(CPUArchState *env, struct target_sigcontext *sc)
{
    target_ulong psw;
    int i;

    __get_user(psw, &sc->sc_gr[0]);
    cpu_hppa_put_psw(env, psw);

    for (i = 1; i < 32; ++i) {
        __get_user(env->gr[i], &sc->sc_gr[i]);
    }
    for (i = 0; i < 32; ++i) {
        __get_user(env->fr[i], &sc->sc_fr[i]);
    }
    cpu_hppa_loaded_fr0(env);

    __get_user(env->iaoq_f, &sc->sc_iaoq[0]);
    __get_user(env->iaoq_b, &sc->sc_iaoq[1]);
    __get_user(env->cr[CR_SAR], &sc->sc_sar);
}

/* No, this doesn't look right, but it's copied straight from the kernel.  */
#define PARISC_RT_SIGFRAME_SIZE32 \
    ((sizeof(struct target_rt_sigframe) + 48 + 64) & -64)

static void setup_rt_frame(int sig, struct target_sigaction *ka,
                           target_siginfo_t *info,
                           target_sigset_t *set, CPUArchState *env)
{
    abi_ulong frame_addr, sp, haddr;
    struct target_rt_sigframe *frame;
    int i;

    sp = env->gr[30];
    if (ka->sa_flags & TARGET_SA_ONSTACK) {
        if (sas_ss_flags(sp) == 0) {
            sp = (target_sigaltstack_used.ss_sp + 0x7f) & ~0x3f;
        }
    }
    frame_addr = QEMU_ALIGN_UP(sp, 64);
    sp = frame_addr + PARISC_RT_SIGFRAME_SIZE32;

    trace_user_setup_rt_frame(env, frame_addr);

    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    tswap_siginfo(&frame->info, info);
    frame->uc.tuc_flags = 0;
    frame->uc.tuc_link = 0;

    __put_user(target_sigaltstack_used.ss_sp, &frame->uc.tuc_stack.ss_sp);
    __put_user(sas_ss_flags(get_sp_from_cpustate(env)),
               &frame->uc.tuc_stack.ss_flags);
    __put_user(target_sigaltstack_used.ss_size,
               &frame->uc.tuc_stack.ss_size);

    for (i = 0; i < TARGET_NSIG_WORDS; i++) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }

    setup_sigcontext(&frame->uc.tuc_mcontext, env);

    __put_user(0x34190000, frame->tramp + 0); /* ldi 0,%r25 */
    __put_user(0x3414015a, frame->tramp + 1); /* ldi __NR_rt_sigreturn,%r20 */
    __put_user(0xe4008200, frame->tramp + 2); /* be,l 0x100(%sr2,%r0) */
    __put_user(0x08000240, frame->tramp + 3); /* nop */

    unlock_user_struct(frame, frame_addr, 1);

    env->gr[2] = h2g(frame->tramp);
    env->gr[30] = sp;
    env->gr[26] = sig;
    env->gr[25] = h2g(&frame->info);
    env->gr[24] = h2g(&frame->uc);

    haddr = ka->_sa_handler;
    if (haddr & 2) {
        /* Function descriptor.  */
        target_ulong *fdesc, dest;

        haddr &= -4;
        if (!lock_user_struct(VERIFY_READ, fdesc, haddr, 1)) {
            goto give_sigsegv;
        }
        __get_user(dest, fdesc);
        __get_user(env->gr[19], fdesc + 1);
        unlock_user_struct(fdesc, haddr, 1);
        haddr = dest;
    }
    env->iaoq_f = haddr;
    env->iaoq_b = haddr + 4;
    return;

 give_sigsegv:
    force_sigsegv(sig);
}

long do_rt_sigreturn(CPUArchState *env)
{
    abi_ulong frame_addr = env->gr[30] - PARISC_RT_SIGFRAME_SIZE32;
    struct target_rt_sigframe *frame;
    sigset_t set;

    trace_user_do_rt_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }
    target_to_host_sigset(&set, &frame->uc.tuc_sigmask);
    set_sigmask(&set);

    restore_sigcontext(env, &frame->uc.tuc_mcontext);
    unlock_user_struct(frame, frame_addr, 0);

    if (do_sigaltstack(frame_addr + offsetof(struct target_rt_sigframe,
                                             uc.tuc_stack),
                       0, env->gr[30]) == -EFAULT) {
        goto badframe;
    }

    unlock_user_struct(frame, frame_addr, 0);
    return -TARGET_QEMU_ESIGRETURN;

 badframe:
    force_sig(TARGET_SIGSEGV);
    return -TARGET_QEMU_ESIGRETURN;
}

#elif defined(TARGET_XTENSA)

struct target_sigcontext {
    abi_ulong sc_pc;
    abi_ulong sc_ps;
    abi_ulong sc_lbeg;
    abi_ulong sc_lend;
    abi_ulong sc_lcount;
    abi_ulong sc_sar;
    abi_ulong sc_acclo;
    abi_ulong sc_acchi;
    abi_ulong sc_a[16];
    abi_ulong sc_xtregs;
};

struct target_ucontext {
    abi_ulong tuc_flags;
    abi_ulong tuc_link;
    target_stack_t tuc_stack;
    struct target_sigcontext tuc_mcontext;
    target_sigset_t tuc_sigmask;
};

struct target_rt_sigframe {
    target_siginfo_t info;
    struct target_ucontext uc;
    /* TODO: xtregs */
    uint8_t retcode[6];
    abi_ulong window[4];
};

static abi_ulong get_sigframe(struct target_sigaction *sa,
                              CPUXtensaState *env,
                              unsigned long framesize)
{
    abi_ulong sp = env->regs[1];

    /* This is the X/Open sanctioned signal stack switching.  */
    if ((sa->sa_flags & TARGET_SA_ONSTACK) != 0 && !sas_ss_flags(sp)) {
        sp = target_sigaltstack_used.ss_sp + target_sigaltstack_used.ss_size;
    }
    return (sp - framesize) & -16;
}

static int flush_window_regs(CPUXtensaState *env)
{
    uint32_t wb = env->sregs[WINDOW_BASE];
    uint32_t ws = xtensa_replicate_windowstart(env) >> (wb + 1);
    unsigned d = ctz32(ws) + 1;
    unsigned i;
    int ret = 0;

    for (i = d; i < env->config->nareg / 4; i += d) {
        uint32_t ssp, osp;
        unsigned j;

        ws >>= d;
        xtensa_rotate_window(env, d);

        if (ws & 0x1) {
            ssp = env->regs[5];
            d = 1;
        } else if (ws & 0x2) {
            ssp = env->regs[9];
            ret |= get_user_ual(osp, env->regs[1] - 12);
            osp -= 32;
            d = 2;
        } else if (ws & 0x4) {
            ssp = env->regs[13];
            ret |= get_user_ual(osp, env->regs[1] - 12);
            osp -= 48;
            d = 3;
        } else {
            g_assert_not_reached();
        }

        for (j = 0; j < 4; ++j) {
            ret |= put_user_ual(env->regs[j], ssp - 16 + j * 4);
        }
        for (j = 4; j < d * 4; ++j) {
            ret |= put_user_ual(env->regs[j], osp - 16 + j * 4);
        }
    }
    xtensa_rotate_window(env, d);
    g_assert(env->sregs[WINDOW_BASE] == wb);
    return ret == 0;
}

static int setup_sigcontext(struct target_rt_sigframe *frame,
                            CPUXtensaState *env)
{
    struct target_sigcontext *sc = &frame->uc.tuc_mcontext;
    int i;

    __put_user(env->pc, &sc->sc_pc);
    __put_user(env->sregs[PS], &sc->sc_ps);
    __put_user(env->sregs[LBEG], &sc->sc_lbeg);
    __put_user(env->sregs[LEND], &sc->sc_lend);
    __put_user(env->sregs[LCOUNT], &sc->sc_lcount);
    if (!flush_window_regs(env)) {
        return 0;
    }
    for (i = 0; i < 16; ++i) {
        __put_user(env->regs[i], sc->sc_a + i);
    }
    __put_user(0, &sc->sc_xtregs);
    /* TODO: xtregs */
    return 1;
}

static void setup_rt_frame(int sig, struct target_sigaction *ka,
                           target_siginfo_t *info,
                           target_sigset_t *set, CPUXtensaState *env)
{
    abi_ulong frame_addr;
    struct target_rt_sigframe *frame;
    uint32_t ra;
    int i;

    frame_addr = get_sigframe(ka, env, sizeof(*frame));
    trace_user_setup_rt_frame(env, frame_addr);

    if (!lock_user_struct(VERIFY_WRITE, frame, frame_addr, 0)) {
        goto give_sigsegv;
    }

    if (ka->sa_flags & SA_SIGINFO) {
        tswap_siginfo(&frame->info, info);
    }

    __put_user(0, &frame->uc.tuc_flags);
    __put_user(0, &frame->uc.tuc_link);
    __put_user(target_sigaltstack_used.ss_sp,
               &frame->uc.tuc_stack.ss_sp);
    __put_user(sas_ss_flags(env->regs[1]),
               &frame->uc.tuc_stack.ss_flags);
    __put_user(target_sigaltstack_used.ss_size,
               &frame->uc.tuc_stack.ss_size);
    if (!setup_sigcontext(frame, env)) {
        unlock_user_struct(frame, frame_addr, 0);
        goto give_sigsegv;
    }
    for (i = 0; i < TARGET_NSIG_WORDS; ++i) {
        __put_user(set->sig[i], &frame->uc.tuc_sigmask.sig[i]);
    }

    if (ka->sa_flags & TARGET_SA_RESTORER) {
        ra = ka->sa_restorer;
    } else {
        ra = frame_addr + offsetof(struct target_rt_sigframe, retcode);
#ifdef TARGET_WORDS_BIGENDIAN
        /* Generate instruction:  MOVI a2, __NR_rt_sigreturn */
        __put_user(0x22, &frame->retcode[0]);
        __put_user(0x0a, &frame->retcode[1]);
        __put_user(TARGET_NR_rt_sigreturn, &frame->retcode[2]);
        /* Generate instruction:  SYSCALL */
        __put_user(0x00, &frame->retcode[3]);
        __put_user(0x05, &frame->retcode[4]);
        __put_user(0x00, &frame->retcode[5]);
#else
        /* Generate instruction:  MOVI a2, __NR_rt_sigreturn */
        __put_user(0x22, &frame->retcode[0]);
        __put_user(0xa0, &frame->retcode[1]);
        __put_user(TARGET_NR_rt_sigreturn, &frame->retcode[2]);
        /* Generate instruction:  SYSCALL */
        __put_user(0x00, &frame->retcode[3]);
        __put_user(0x50, &frame->retcode[4]);
        __put_user(0x00, &frame->retcode[5]);
#endif
    }
    env->sregs[PS] = PS_UM | (3 << PS_RING_SHIFT);
    if (xtensa_option_enabled(env->config, XTENSA_OPTION_WINDOWED_REGISTER)) {
        env->sregs[PS] |= PS_WOE | (1 << PS_CALLINC_SHIFT);
    }
    memset(env->regs, 0, sizeof(env->regs));
    env->pc = ka->_sa_handler;
    env->regs[1] = frame_addr;
    env->sregs[WINDOW_BASE] = 0;
    env->sregs[WINDOW_START] = 1;

    env->regs[4] = (ra & 0x3fffffff) | 0x40000000;
    env->regs[6] = sig;
    env->regs[7] = frame_addr + offsetof(struct target_rt_sigframe, info);
    env->regs[8] = frame_addr + offsetof(struct target_rt_sigframe, uc);
    unlock_user_struct(frame, frame_addr, 1);
    return;

give_sigsegv:
    force_sigsegv(sig);
    return;
}

static void restore_sigcontext(CPUXtensaState *env,
                               struct target_rt_sigframe *frame)
{
    struct target_sigcontext *sc = &frame->uc.tuc_mcontext;
    uint32_t ps;
    int i;

    __get_user(env->pc, &sc->sc_pc);
    __get_user(ps, &sc->sc_ps);
    __get_user(env->sregs[LBEG], &sc->sc_lbeg);
    __get_user(env->sregs[LEND], &sc->sc_lend);
    __get_user(env->sregs[LCOUNT], &sc->sc_lcount);

    env->sregs[WINDOW_BASE] = 0;
    env->sregs[WINDOW_START] = 1;
    env->sregs[PS] = deposit32(env->sregs[PS],
                               PS_CALLINC_SHIFT,
                               PS_CALLINC_LEN,
                               extract32(ps, PS_CALLINC_SHIFT,
                                         PS_CALLINC_LEN));
    for (i = 0; i < 16; ++i) {
        __get_user(env->regs[i], sc->sc_a + i);
    }
    /* TODO: xtregs */
}

long do_rt_sigreturn(CPUXtensaState *env)
{
    abi_ulong frame_addr = env->regs[1];
    struct target_rt_sigframe *frame;
    sigset_t set;

    trace_user_do_rt_sigreturn(env, frame_addr);
    if (!lock_user_struct(VERIFY_READ, frame, frame_addr, 1)) {
        goto badframe;
    }
    target_to_host_sigset(&set, &frame->uc.tuc_sigmask);
    set_sigmask(&set);

    restore_sigcontext(env, frame);

    if (do_sigaltstack(frame_addr +
                       offsetof(struct target_rt_sigframe, uc.tuc_stack),
                       0, get_sp_from_cpustate(env)) == -TARGET_EFAULT) {
        goto badframe;
    }
    unlock_user_struct(frame, frame_addr, 0);
    return -TARGET_QEMU_ESIGRETURN;

badframe:
    unlock_user_struct(frame, frame_addr, 0);
    force_sig(TARGET_SIGSEGV);
    return -TARGET_QEMU_ESIGRETURN;
}
#endif

static void handle_pending_signal(CPUArchState *cpu_env, int sig,
                                  struct emulated_sigtable *k)
{
    CPUState *cpu = ENV_GET_CPU(cpu_env);
    abi_ulong handler;
    sigset_t set;
    target_sigset_t target_old_set;
    struct target_sigaction *sa;
    TaskState *ts = cpu->opaque;

    trace_user_handle_signal(cpu_env, sig);
    /* dequeue signal */
    k->pending = 0;

    sig = gdb_handlesig(cpu, sig);
    if (!sig) {
        sa = NULL;
        handler = TARGET_SIG_IGN;
    } else {
        sa = &sigact_table[sig - 1];
        handler = sa->_sa_handler;
    }

    if (do_strace) {
        print_taken_signal(sig, &k->info);
    }

    if (handler == TARGET_SIG_DFL) {
        /* default handler : ignore some signal. The other are job control or fatal */
        if (sig == TARGET_SIGTSTP || sig == TARGET_SIGTTIN || sig == TARGET_SIGTTOU) {
            kill(getpid(),SIGSTOP);
        } else if (sig != TARGET_SIGCHLD &&
                   sig != TARGET_SIGURG &&
                   sig != TARGET_SIGWINCH &&
                   sig != TARGET_SIGCONT) {
            dump_core_and_abort(sig);
        }
    } else if (handler == TARGET_SIG_IGN) {
        /* ignore sig */
    } else if (handler == TARGET_SIG_ERR) {
        dump_core_and_abort(sig);
    } else {
        /* compute the blocked signals during the handler execution */
        sigset_t *blocked_set;

        target_to_host_sigset(&set, &sa->sa_mask);
        /* SA_NODEFER indicates that the current signal should not be
           blocked during the handler */
        if (!(sa->sa_flags & TARGET_SA_NODEFER))
            sigaddset(&set, target_to_host_signal(sig));

        /* save the previous blocked signal state to restore it at the
           end of the signal execution (see do_sigreturn) */
        host_to_target_sigset_internal(&target_old_set, &ts->signal_mask);

        /* block signals in the handler */
        blocked_set = ts->in_sigsuspend ?
            &ts->sigsuspend_mask : &ts->signal_mask;
        sigorset(&ts->signal_mask, blocked_set, &set);
        ts->in_sigsuspend = 0;

        /* if the CPU is in VM86 mode, we restore the 32 bit values */
#if defined(TARGET_I386) && !defined(TARGET_X86_64)
        {
            CPUX86State *env = cpu_env;
            if (env->eflags & VM_MASK)
                save_v86_state(env);
        }
#endif
        /* prepare the stack frame of the virtual CPU */
#if defined(TARGET_ABI_MIPSN32) || defined(TARGET_ABI_MIPSN64) \
        || defined(TARGET_OPENRISC) || defined(TARGET_TILEGX) \
        || defined(TARGET_PPC64) || defined(TARGET_HPPA) \
        || defined(TARGET_NIOS2) || defined(TARGET_X86_64) \
        || defined(TARGET_RISCV) || defined(TARGET_XTENSA)
        /* These targets do not have traditional signals.  */
        setup_rt_frame(sig, sa, &k->info, &target_old_set, cpu_env);
#else
        if (sa->sa_flags & TARGET_SA_SIGINFO)
            setup_rt_frame(sig, sa, &k->info, &target_old_set, cpu_env);
        else
            setup_frame(sig, sa, &target_old_set, cpu_env);
#endif
        if (sa->sa_flags & TARGET_SA_RESETHAND) {
            sa->_sa_handler = TARGET_SIG_DFL;
        }
    }
}

void process_pending_signals(CPUArchState *cpu_env)
{
    CPUState *cpu = ENV_GET_CPU(cpu_env);
    int sig;
    TaskState *ts = cpu->opaque;
    sigset_t set;
    sigset_t *blocked_set;

    while (atomic_read(&ts->signal_pending)) {
        /* FIXME: This is not threadsafe.  */
        sigfillset(&set);
        sigprocmask(SIG_SETMASK, &set, 0);

    restart_scan:
        sig = ts->sync_signal.pending;
        if (sig) {
            /* Synchronous signals are forced,
             * see force_sig_info() and callers in Linux
             * Note that not all of our queue_signal() calls in QEMU correspond
             * to force_sig_info() calls in Linux (some are send_sig_info()).
             * However it seems like a kernel bug to me to allow the process
             * to block a synchronous signal since it could then just end up
             * looping round and round indefinitely.
             */
            if (sigismember(&ts->signal_mask, target_to_host_signal_table[sig])
                || sigact_table[sig - 1]._sa_handler == TARGET_SIG_IGN) {
                sigdelset(&ts->signal_mask, target_to_host_signal_table[sig]);
                sigact_table[sig - 1]._sa_handler = TARGET_SIG_DFL;
            }

            handle_pending_signal(cpu_env, sig, &ts->sync_signal);
        }

        for (sig = 1; sig <= TARGET_NSIG; sig++) {
            blocked_set = ts->in_sigsuspend ?
                &ts->sigsuspend_mask : &ts->signal_mask;

            if (ts->sigtab[sig - 1].pending &&
                (!sigismember(blocked_set,
                              target_to_host_signal_table[sig]))) {
                handle_pending_signal(cpu_env, sig, &ts->sigtab[sig - 1]);
                /* Restart scan from the beginning, as handle_pending_signal
                 * might have resulted in a new synchronous signal (eg SIGSEGV).
                 */
                goto restart_scan;
            }
        }

        /* if no signal is pending, unblock signals and recheck (the act
         * of unblocking might cause us to take another host signal which
         * will set signal_pending again).
         */
        atomic_set(&ts->signal_pending, 0);
        ts->in_sigsuspend = 0;
        set = ts->signal_mask;
        sigdelset(&set, SIGSEGV);
        sigdelset(&set, SIGBUS);
        sigprocmask(SIG_SETMASK, &set, 0);
    }
    ts->in_sigsuspend = 0;
}
