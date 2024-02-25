extern crate proc_macro;

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

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
        fn main() -> unros::anyhow::Result<()> {
            #input
            unros::start_unros_runtime(
                #ident,
                unros::default_run_options!(),
            )
        }
    }
    .into()
}
