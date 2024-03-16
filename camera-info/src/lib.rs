use std::{
    collections::HashMap,
    num::NonZeroUsize,
    sync::{Arc, OnceLock},
};

use image::DynamicImage;
use serde::{Deserialize, Serialize};
use serde_json::from_reader;
use unros::{log::error, pubsub::Subscription};

static CAMERA_DB: OnceLock<HashMap<String, Arc<CameraInfo>>> = OnceLock::new();

const DEFAULT_CAMERA_FOLDER: &str = "camera-db";

fn get_camera_db() -> &'static HashMap<String, Arc<CameraInfo>> {
    CAMERA_DB.get_or_init(|| {
        let mut map: HashMap<String, Arc<CameraInfo>> = HashMap::default();

        let paths = match std::fs::read_dir(DEFAULT_CAMERA_FOLDER) {
            Ok(x) => x,
            Err(e) => {
                error!("Faced the following error while trying to listdir: {DEFAULT_CAMERA_FOLDER}: {e}");
                return map;
            }
        };

        for path in paths {
            let path = match path {
                Ok(x) => x.path(),
                Err(e) => {
                    error!("Faced the following error while trying to listdir: {DEFAULT_CAMERA_FOLDER}: {e}");
                    continue;
                }
            };
            let file = match std::fs::File::open(&path) {
                Ok(x) => x,
                Err(e) => {
                    error!("Faced the following error while trying to read: {path:?}: {e}");
                    continue;
                }
            };
            let submap: HashMap<String, CameraInfo> = match from_reader(file) {
                Ok(x) => x,
                Err(e) => {
                    error!("Faced the following error while trying to parse: {path:?}: {e}");
                    continue;
                }
            };
            map.extend(submap.into_iter().map(|(a, b)| (a, Arc::new(b))));
        }

        map
    })
}

#[derive(Serialize, Deserialize)]
pub struct CameraInfo {
    pub width: u32,
    pub height: u32,
    #[serde(default)]
    pub fps: Option<NonZeroUsize>,
}

impl CameraInfo {
    pub fn from_name(name: &str) -> Option<Arc<Self>> {
        get_camera_db().get(name).cloned()
    }

    pub fn undistort_subscription(
        &self,
        sub: Subscription<Arc<DynamicImage>>,
    ) -> Subscription<Arc<DynamicImage>> {
        sub.map(|x| x)
    }
}
