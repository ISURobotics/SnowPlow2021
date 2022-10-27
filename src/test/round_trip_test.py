import Light
import rospy

def main():
    """
    This function will light
    """
    rospy.init_node('Testing', anonymous=True)
    light = Light.Light("")
    value = 0
    while value != 3:
        value = input("Enter light value: \n")
        light.set_light(value)
    exit(0)


if __name__ == "__main__":
    main()
