extern crate proc_macro;

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

/// Wraps the given method to be ran using `start_unros_runtime`.
///
/// This can be used on any method, but we will use `main` as an example:
///
/// ```rust
/// fn main() {
///     todo!()
/// }
/// ```
///
/// The above code needs to be converted to the below code:
///
/// ```rust
/// #[unros::main]
/// async fn main(app: unros::Application) -> unros::anyhow::Result<unros::Application> {
///     todo!()
/// }
/// ```
///
/// Refer to the [book](https://utahrobotics.github.io/unros-book/hello-goodbye.html) for more information.
#[proc_macro_attribute]
pub fn main(attr: TokenStream, item: TokenStream) -> TokenStream {
    if !attr.is_empty() {
        return quote! { compile_error!("unros::main does not accept attributes"); }.into();
    }
    let input: ItemFn = parse_macro_input!(item);
    let ident = input.sig.ident.clone();

    if input.sig.asyncness.is_none() {
        return quote! { compile_error!("The function must be `async`"); }.into();
    }

    if input.sig.inputs.len() != 1 {
        return quote! { compile_error!("The function must have exactly 1 parameter"); }.into();
    }

    // let param = input.sig.inputs.first_mut().unwrap();
    // let FnArg::Typed(ref mut param) = param else { panic!("Invalid parameter"); };
    // if let Type::Infer(_) = param.ty.deref() {
    //     param.ty = Box::new(Type::Verbatim(quote!{ unros::Application }));
    // }

    // match &input.sig.output {
    //     ReturnType::Type(_, ty) => match ty.deref() {
    //         Type::Path(x) => panic!("{x:?}"),
    //         _ => panic!("The method must return an `Application` object"),
    //     }
    //     _ => panic!("The method must return an `Application` object"),
    // }

    quote! {
        fn main() -> std::process::ExitCode {
            #input
            let Some(result) = unros::start_unros_runtime(
                #ident,
                |builder| {
                    builder.dump_path = unros::runtime::DumpPath {
                        Default {
                            application_name: env!("CARGO_PKG_NAME").into()
                        }
                    };
                },
            ) else {
                return std::process::ExitCode::SUCCESS;
            };

            std::process::Termination::report(result)
        }
    }
    .into()
}
