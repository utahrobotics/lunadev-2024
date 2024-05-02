def main():
    while True:
        try:
            text = input('')
            exec(text)
            print(">>>END=REP<<<")
        except EOFError:
            break
        except Exception as e:
            print(e)
            print(">>>END=REP<<<")

if __name__ == '__main__':
    main()