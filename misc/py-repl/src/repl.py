def __end_repl__():
    print(">>>END=REPL<<<")


def main():
    __text = ""
    while True:
        __text += input('') + "\n"
        try:
            if __text.endswith("__end_repl__()\n"):
                exec(__text)
                __text = ""
        except EOFError:
            break
        except Exception as e:
            print(e)
            __end_repl__()

if __name__ == '__main__':
    main()
