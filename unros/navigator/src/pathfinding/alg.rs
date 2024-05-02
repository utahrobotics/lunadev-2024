use std::{
    cmp::Ordering,
    collections::BinaryHeap,
    hash::{BuildHasherDefault, Hash},
    ops::Add,
};

use fxhash::FxHasher;
use indexmap::{map::Entry, IndexMap};

type FxIndexMap<K, V> = IndexMap<K, V, BuildHasherDefault<FxHasher>>;

fn reverse_path<N, V, F>(parents: &FxIndexMap<N, V>, mut parent: F, start: usize) -> Vec<N>
where
    N: Eq + Hash + Clone,
    F: FnMut(&V) -> usize,
{
    let mut i = start;
    let path = std::iter::from_fn(|| {
        parents.get_index(i).map(|(node, value)| {
            i = parent(value);
            node
        })
    })
    .collect::<Vec<&N>>();
    // Collecting the going through the vector is needed to revert the path because the
    // unfold iterator is not double-ended due to its iterative nature.
    path.into_iter().rev().cloned().collect()
}

struct SmallestCostHolder<K> {
    estimated_cost: K,
    cost: K,
    index: usize,
}

impl<K: PartialEq> PartialEq for SmallestCostHolder<K> {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost.eq(&other.estimated_cost) && self.cost.eq(&other.cost)
    }
}

impl<K: PartialEq> Eq for SmallestCostHolder<K> {}

impl<K: Ord> PartialOrd for SmallestCostHolder<K> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K: Ord> Ord for SmallestCostHolder<K> {
    fn cmp(&self, other: &Self) -> Ordering {
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            s => s,
        }
    }
}

pub trait AStarModule<N, C> {
    async fn successors(&mut self, node: N, out: impl FnMut(N, C));
    async fn success(&mut self, node: &N) -> bool;
}

pub async fn astar<N, C, FN, FH>(
    start: &N,
    module: &mut FN,
    mut heuristic: FH,
) -> Option<(Vec<N>, C)>
where
    N: Eq + Hash + Clone,
    C: Default + Ord + Add<Output = C> + Copy,
    FN: AStarModule<N, C>,
    FH: FnMut(&N) -> C,
{
    let mut to_see = BinaryHeap::new();
    to_see.push(SmallestCostHolder {
        estimated_cost: C::default(),
        cost: C::default(),
        index: 0,
    });
    let mut parents: FxIndexMap<N, (usize, C)> = FxIndexMap::default();
    parents.insert(start.clone(), (usize::max_value(), C::default()));
    while let Some(SmallestCostHolder { cost, index, .. }) = to_see.pop() {
        let (node, &(_, c)) = parents.get_index(index).unwrap(); // Cannot fail
        if module.success(node).await {
            let path = reverse_path(&parents, |&(p, _)| p, index);
            return Some((path, cost));
        }
        // We may have inserted a node several time into the binary heap if we found
        // a better way to access it. Ensure that we are currently dealing with the
        // best path and discard the others.
        if cost > c {
            continue;
        }
        module
            .successors(node.clone(), |successor, move_cost| {
                let new_cost = cost + move_cost;
                let h; // heuristic(&successor)
                let n; // index for successor
                match parents.entry(successor) {
                    Entry::Vacant(e) => {
                        h = heuristic(e.key());
                        n = e.index();
                        e.insert((index, new_cost));
                    }
                    Entry::Occupied(mut e) => {
                        if e.get().1 > new_cost {
                            h = heuristic(e.key());
                            n = e.index();
                            e.insert((index, new_cost));
                        } else {
                            return;
                        }
                    }
                }

                to_see.push(SmallestCostHolder {
                    estimated_cost: new_cost + h,
                    cost: new_cost,
                    index: n,
                });
            })
            .await;
    }
    None
}
