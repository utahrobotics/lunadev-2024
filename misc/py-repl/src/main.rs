use py_repl::PyRepl;

fn main() -> anyhow::Result<()> {
    let mut py_repl = PyRepl::new(".")?;
    println!("{}", py_repl.exec(r#"print("hello world")"#)?);
    let stderr = py_repl.read_stderr_to_end()?;
    if !stderr.is_empty() {
        eprintln!("\nStderr:\n{stderr}");
    }
    Ok(())
}
