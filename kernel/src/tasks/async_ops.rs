use core::{future::Future, pin::Pin, task::Poll};

use alloc::sync::Arc;
use arch::get_time_ms;
use executor::UserTask;

pub struct NextTick(usize);

impl Future for NextTick {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut core::task::Context<'_>) -> Poll<Self::Output> {
        let curr = get_time_ms();
        if curr < self.0 {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}

pub struct WaitPid(pub Arc<UserTask>, pub isize);

impl Future for WaitPid {
    type Output = Arc<UserTask>;

    fn poll(self: Pin<&mut Self>, _cx: &mut core::task::Context<'_>) -> Poll<Self::Output> {
        let inner = self.0.inner.lock();
        let res = inner.children.iter().find(|x| {
            let inner = x.inner.lock();
            (self.1 == -1 || x.task_id == self.1 as usize) && inner.exit_code.is_some()
        });
        match res {
            Some(task) => Poll::Ready(task.clone()),
            None => Poll::Pending,
        }
    }
}