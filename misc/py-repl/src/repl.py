def __end_repl__():
    print(">>>END=REPL<<<")


def main():
    __text = ""
    while True:
        try:
            __text += input('') + "\n"
        except EOFError:
            break

        try:
            if __text.endswith("__end_repl__()\n"):
                exec(__text)
                __text = ""
        except Exception as e:
            print(e)
            __end_repl__()

if __name__ == '__main__':
    main()
