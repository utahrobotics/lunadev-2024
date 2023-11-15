use std::{
    fs::File,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    sync::mpsc,
};

use log::error;

use crate::SUB_LOGGING_DIR;

struct DataDumpInner {
    writer: mpsc::Sender<Vec<u8>>,
    empty_vecs: mpsc::Receiver<Vec<u8>>,
}

pub struct DataDump(Option<DataDumpInner>);

impl DataDump {
    pub async fn new(filename: impl AsRef<Path>) -> std::io::Result<Self> {
        let Some(path) = SUB_LOGGING_DIR.get() else {
            return Ok(Self(None));
        };
        let filename = PathBuf::from(filename.as_ref());
        let file = File::create(PathBuf::from(path).join(&filename))?;
        let (empty_vecs_sender, empty_vecs) = mpsc::channel();
        let (writer, reader) = mpsc::channel::<Vec<_>>();
        std::thread::spawn(move || {
            let mut file = BufWriter::new(file);
            loop {
                let Ok(mut bytes) = reader.recv() else {
                    break;
                };
                if let Err(e) = file.write_all(&bytes) {
                    error!("Failed to write to {filename:?}: {e}");
                    return;
                }
                bytes.clear();
                let _ = empty_vecs_sender.send(bytes);
            }
            if let Err(e) = file.flush() {
                error!("Failed to flush to {filename:?}: {e}");
            }
        });
        Ok(Self(Some(DataDumpInner { writer, empty_vecs })))
    }
}

impl Write for DataDump {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let Some(inner) = self.0.as_mut() else {
            return Ok(buf.len());
        };
        let mut vec = inner.empty_vecs.try_recv().unwrap_or_default();
        vec.extend_from_slice(buf);
        inner
            .writer
            .send(vec)
            .map_err(|_| std::io::Error::from(std::io::ErrorKind::BrokenPipe))?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
