//! run_batch over a local rayon pool (feature parallel) returning results in index order. Owner: A7 (complete in Wave 0, W0-B3).
//!
//! Results are always in input order, so ensembles are identical for any worker count
//! (core design §A.4 item 5). Without the `parallel` feature (e.g. the wasm32 build of
//! `fabrail-web`) everything runs sequentially on the calling thread.

use fabrail_params::SystemParams;

use crate::recorder::RunData;
use crate::scenario::{Scenario, SimError};
use crate::scheduler::run;

/// Maps `f(index, item)` over `items`, returning results in index order. `workers`:
/// `None` or `Some(0)` = rayon's default thread count, `Some(1)` = sequential,
/// `Some(n)` = a local pool of n threads. Falls back to sequential if the pool cannot
/// be built or the `parallel` feature is off.
pub fn map_ordered<T, R, F>(items: &[T], workers: Option<usize>, f: F) -> Vec<R>
where
    T: Sync,
    R: Send,
    F: Fn(usize, &T) -> R + Sync + Send,
{
    #[cfg(feature = "parallel")]
    {
        let n = workers.unwrap_or(0);
        if n != 1 && items.len() > 1 {
            let mut b = rayon::ThreadPoolBuilder::new();
            if n > 0 {
                b = b.num_threads(n);
            }
            if let Ok(pool) = b.build() {
                use rayon::prelude::*;
                return pool
                    .install(|| items.par_iter().enumerate().map(|(i, t)| f(i, t)).collect());
            }
        }
    }
    let _ = workers;
    items.iter().enumerate().map(|(i, t)| f(i, t)).collect()
}

/// Runs every job, results in job order (core design §B.6).
pub fn run_batch(
    jobs: &[(SystemParams, Scenario)],
    workers: Option<usize>,
) -> Vec<Result<RunData, SimError>> {
    map_ordered(jobs, workers, |_, (p, s)| run(p, s))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn results_are_in_index_order_for_any_worker_count() {
        let items: Vec<u64> = (0..257).collect();
        let reference: Vec<u64> = items.iter().map(|x| x * x + 1).collect();
        for w in [None, Some(0), Some(1), Some(2), Some(7)] {
            assert_eq!(
                map_ordered(&items, w, |_, x| x * x + 1),
                reference,
                "workers {w:?}"
            );
        }
        let idx = map_ordered(&items, Some(3), |i, _| i);
        assert!(idx.iter().enumerate().all(|(k, &i)| k == i));
        assert!(map_ordered::<u64, u64, _>(&[], Some(4), |_, x| *x).is_empty());
    }

    #[test]
    fn run_batch_keeps_one_result_per_job() {
        assert!(run_batch(&[], None).is_empty());
    }
}
