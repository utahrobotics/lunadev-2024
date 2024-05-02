use std::{
    io::{Read, Write},
    process::{Child, ChildStderr, ChildStdin, ChildStdout, Command, Stdio},
};

const TERMINATOR: &'static [u8] = b">>>END=REP<<<\n";

pub struct PyRepl {
    cmd: Child,
    stdin: Option<ChildStdin>,
    stdout: ChildStdout,
    stderr: ChildStderr,
}

impl PyRepl {
    pub fn new(working_dir: &str) -> anyhow::Result<Self> {
        // let mut imports_block = String::new();
        // for import in imports {
        //     imports_block.push_str("import ");
        //     imports_block.push_str(import);
        //     imports_block.push_str("\n");
        // }
        let mut cmd = Command::new("python3")
            .current_dir(working_dir)
            .arg("-c")
            .arg(include_str!("repl.py"))
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .stdin(Stdio::piped())
            .spawn()?;

        Ok(Self {
            stdin: cmd.stdin.take(),
            stdout: cmd.stdout.take().unwrap(),
            stderr: cmd.stderr.take().unwrap(),
            cmd,
        })
    }

    pub fn close(&mut self) -> anyhow::Result<()> {
        self.stdin = None;
        self.cmd.kill().map_err(Into::into)
    }

    pub fn exec(&mut self, code: &str) -> anyhow::Result<String> {
        let stdin = self.stdin.as_mut().unwrap();
        stdin.write_all(code.as_bytes())?;
        stdin.write_all(b"\n")?;
        stdin.flush()?;

        let mut returned = vec![];
        let mut buf = [0; 1024];
        loop {
            let n = self.stdout.read(&mut buf)?;
            if n == 0 {
                return String::from_utf8(returned).map_err(Into::into);
            }
            returned.extend_from_slice(buf.split_at(n).0);
            if returned.ends_with(TERMINATOR) {
                break;
            }
        }
        returned.drain((returned.len() - TERMINATOR.len()).saturating_sub(1)..);
        String::from_utf8(returned).map_err(Into::into)
    }

    pub fn read_stderr_to_end(&mut self) -> anyhow::Result<String> {
        self.close()?;
        let mut returned = String::new();
        self.stderr.read_to_string(&mut returned)?;
        Ok(returned)
    }
}
