import cv2
import os

def test_drone_vision():
    print("Booting Vision System...")
    
    # The path to the left camera's image folder
    cam0_path = r"C:\Users\phani\Drone_SLAM\mav0\cam0\data"
    
    if not os.path.exists(cam0_path):
        print("ERROR: Camera folder not found!")
        return

    # Get a list of all the images in the folder and sort them by timestamp
    image_files = sorted(os.listdir(cam0_path))
    
    if len(image_files) == 0:
        print("ERROR: No images found in the folder!")
        return
        
    # Grab the very first image file
    first_image_name = image_files[0]
    first_image_path = os.path.join(cam0_path, first_image_name)
    
    print(f"Loading first frame: {first_image_name}")
    
    # Read the image using OpenCV
    img = cv2.imread(first_image_path, cv2.IMREAD_GRAYSCALE)
    
    # Display the image
    cv2.imshow("Drone Left Eye (cam0)", img)
    
    print("Press ANY KEY on your keyboard to close the image window.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_drone_vision()