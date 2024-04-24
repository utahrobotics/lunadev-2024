use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput, ItemFn};

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
        fn main() {
            #input
            unros::runtime::start_unros_runtime_main_thread(
                #ident,
                |builder| {
                    builder.dump_path = unros::runtime::DumpPath::Default {
                        application_name: env!("CARGO_PKG_NAME").into()
                    };
                },
            )
        }
    }
    .into()
}

#[proc_macro_derive(ShouldNotDrop)]
pub fn my_macro(input: TokenStream) -> TokenStream {
    // Parse the input tokens into a syntax tree
    let input = parse_macro_input!(input as DeriveInput);

    let name = input.ident;
    let generics = input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    // Build the output, possibly using quasi-quotation
    let expanded = quote! {
        impl #impl_generics unros::ShouldNotDrop for #name #ty_generics #where_clause {
            fn get_dont_drop(&mut self) -> &mut unros::DontDrop<Self> {
                &mut self.dont_drop
            }
        }
    };

    // Hand the output tokens back to the compiler
    TokenStream::from(expanded)
}
