use data::{gen, gen_rf_data};
use smartcore::{ensemble::random_forest_regressor::{RandomForestRegressor, RandomForestRegressorSearchParameters}, linalg::basic::matrix::DenseMatrix};

mod data;

const SCALE: f32 = 2.0;
const THRESHOLD: f32 = 0.1;
const TEST_N: usize = 1000;

fn main() {
    let (x, y) = gen_rf_data(5000);
    let mut best_so_far = 0.0;
    let params = RandomForestRegressorSearchParameters {
        max_depth: vec![Some(15), Some(20), Some(30), None],
        min_samples_leaf: vec![1, 2, 3, 4],
        min_samples_split: vec![2, 3, 4, 5],
        n_trees: vec![5, 7, 10],
        ..Default::default()
    };
    let mut best_param = None;
    
    for (i, param) in params.into_iter().enumerate() {
        println!("{i} {param:?}");
        if best_param.is_none() {
            best_param = Some(param.clone());
        }
        let regressor = RandomForestRegressor::fit(&x, &y, param.clone()).unwrap();

        let mut rand_wins = 0usize;
        let mut model_wins = 0usize;
        let mut ties = 0usize;
    
        for _ in 0..TEST_N {
            let item = gen();
            let x = DenseMatrix::from_2d_vec(&vec![item.input.into_iter().flat_map(|(x, v, d)| [x, v, d]).collect()]);
            let y_hat = regressor.predict(&x).unwrap();
            let model_won = (item.target - y_hat[0]).abs() * SCALE <= THRESHOLD;
            let rand_won = (item.target - item.input.last().unwrap().0).abs() * SCALE <= THRESHOLD;
            if model_won && rand_won {
                ties += 1;
            } else if model_won {
                model_wins += 1;
            } else if rand_won {
                rand_wins += 1;
            }
        }
    
        let model_score = (model_wins * 100) as f32 / TEST_N as f32;
        let rand_score = (rand_wins * 100) as f32 / TEST_N as f32;
        let ties_score = (ties * 100) as f32 / TEST_N as f32;
        println!("model_score: {model_score}% rand_score: {rand_score}% ties_score: {ties_score}%\n");
        
        if model_score > best_so_far {
            let data = bincode::serialize(&regressor).unwrap();
            std::fs::write("localization/src/model.bin", data).unwrap();
            best_so_far = model_score;
            best_param = Some(param);
        }
    }

    println!("best score: {best_so_far}% from {:?}", best_param.unwrap());
}