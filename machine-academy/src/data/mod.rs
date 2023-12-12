use std::{collections::VecDeque, fs::File, path::PathBuf, sync::Mutex};

use burn::data::dataset::Dataset;
use crossbeam::queue::SegQueue;
use rand::{rngs::SmallRng, Rng, SeedableRng};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use tokio_rayon::rayon::iter::{IntoParallelIterator, ParallelIterator};

struct CacheBlock<T> {
    items: Mutex<Box<[T]>>,
}

#[derive(Serialize, Deserialize)]
struct AcademyDatasetConfig {
    block_memory_size: usize,
    block_count: usize,
    block_size: usize,
    length: usize,
}

pub struct AcademyDataset<T> {
    cache: Box<[CacheBlock<T>]>,
    last_used: Mutex<VecDeque<usize>>,
    length: usize,
    block_size: usize,
    max_cached_blocks: usize,
    data_path: PathBuf,
}

impl<T> AcademyDataset<T> {
    pub fn new(data_path: PathBuf, max_memory_usage: usize) -> Self {
        let config: AcademyDatasetConfig = bincode::deserialize_from(
            File::open(data_path.join("config.dat")).expect("config.dat should be readable"),
        )
        .expect("config.dat should be valid");

        Self {
            cache: (0..config.block_count)
                .into_iter()
                .map(|_| CacheBlock {
                    items: Mutex::new(Box::new([])),
                })
                .collect(),
            last_used: Mutex::new(VecDeque::with_capacity(config.block_count)),
            length: config.length,
            block_size: config.block_size,
            max_cached_blocks: (max_memory_usage / config.block_memory_size).max(1),
            data_path,
        }
    }
}

impl<T: DeserializeOwned + Send + Sync + Clone> Dataset<T> for AcademyDataset<T> {
    fn get(&self, index: usize) -> Option<T> {
        let block_index = index / self.block_size;
        let block = self.cache.get(block_index)?;
        let mut items = block.items.lock().unwrap();
        let mut last_used = self.last_used.lock().unwrap();

        if items.is_empty() {
            if last_used.len() >= self.max_cached_blocks {
                let last_used_index = last_used.pop_back().unwrap();
                *self
                    .cache
                    .get(last_used_index)
                    .unwrap()
                    .items
                    .lock()
                    .unwrap() = Box::new([]);
            }
            last_used.push_front(block_index);
            let filename = format!("{block_index}.slice");
            let slice_path = self.data_path.join(filename);

            *items = bincode::deserialize_from(
                File::open(slice_path).expect("Database slice should be readable"),
            )
            .expect("Database slice should be valid");
        } else {
            let mut tmp_indices = Vec::with_capacity(self.max_cached_blocks);
            loop {
                let Some(i) = last_used.pop_front() else {
                    break;
                };
                if i == block_index {
                    break;
                }
                tmp_indices.push(i);
            }
            for i in tmp_indices.into_iter().rev() {
                last_used.push_front(i);
            }
            last_used.push_front(block_index);
        }

        items.get(block_index % self.block_size).cloned()
    }

    fn len(&self) -> usize {
        self.length
    }
}

pub fn create_dataset<T>(length: usize, data_path: PathBuf, block_memory_size: usize)
where
    T: for<'a> From<&'a mut SmallRng> + Serialize + Send,
{
    std::fs::create_dir_all(&data_path).expect("Data path directories should be creatable");
    let mut first_rng = SmallRng::from_entropy();
    let mut first_block = vec![];
    let mut block_size = 0usize;

    for _ in 0..length {
        first_block.push(T::from(&mut first_rng));
        block_size += 1;
        if bincode::serialized_size(&first_block).expect("Type T should be serializable") as usize
            >= block_memory_size
        {
            break;
        }
    }
    bincode::serialize_into(
        File::create(data_path.join("0.slice")).expect("Database slice should be creatable"),
        &first_block,
    )
    .expect("Database slice should be writable, and the type T should be serializable");

    if block_size >= length {
        let config = AcademyDatasetConfig {
            block_memory_size,
            block_count: 1,
            block_size,
            length,
        };
    
        bincode::serialize_into(
            File::create(data_path.join("config.dat")).expect("Database config should be creatable"),
            &config,
        )
        .expect("Database config should be writable");
        return;
    }

    let remaining_block_count = (length - block_size) / block_size;
    let small_block_count = (length - block_size) % block_size;
    first_block.clear();

    if small_block_count > 0 {
        for _ in 0..small_block_count {
            first_block.push(T::from(&mut first_rng));
        }
        bincode::serialize_into(
            File::create(data_path.join(format!("{}.slice", remaining_block_count + 1)))
                .expect("Database slice should be creatable"),
            &first_block,
        )
        .expect("Database slice should be writable, and the type T should be serializable");
    }
    drop(first_block);

    let block_count = if small_block_count > 0 {
        remaining_block_count + 2
    } else {
        remaining_block_count + 1
    };

    {
        let mut i = block_count;
        loop {
            let path = data_path.join(format!("{i}.slice"));
            if path
                .try_exists()
                .expect("Files in data path should be readable")
            {
                std::fs::remove_file(path).expect("Files in data path should be deletable");
            } else {
                break;
            }
            i += 1;
        }
    }

    let rands = SegQueue::new();
    rands.push(first_rng);

    (1..(remaining_block_count + 1))
        .into_par_iter()
        .for_each(|i| {
            let block: Box<[T]> = (0..block_size)
                .into_par_iter()
                .map(|_| {
                    let mut rng = rands.pop().unwrap_or_else(|| SmallRng::from_entropy());
                    let item = T::from(&mut rng);
                    // 1% chance of resampling the rng
                    if rng.gen_bool(0.01) {
                        rng = SmallRng::from_entropy();
                    }
                    rands.push(rng);
                    item
                })
                .collect();
            bincode::serialize_into(
                File::create(data_path.join(format!("{i}.slice")))
                    .expect("Database slice should be creatable"),
                &block,
            )
            .expect("Database slice should be writable, and the type T should be serializable");
        });

    let config = AcademyDatasetConfig {
        block_memory_size,
        block_count,
        block_size,
        length,
    };

    bincode::serialize_into(
        File::create(data_path.join("config.dat")).expect("Database config should be creatable"),
        &config,
    )
    .expect("Database config should be writable");
}
