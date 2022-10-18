import Light


def main():
    """
    This function will light
    """
    light = Light.Light("")
    value = 0
    while value != 4:
        value = input("Enter light value: \n")
        light.set_light(value)
    exit(0)


if __name__ == "__main__":
    main()
