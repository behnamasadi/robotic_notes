import rerun as rr
import cv2
import os
from datetime import datetime

def display_images_with_timeline(image_folder):
    """
    Displays images with their timestamps in Rerun.

    :param image_folder: Path to the folder containing images.
    """
    # Initialize Rerun
    rr.init("image_timeline", spawn=True)

    # Get the list of image files in the folder
    image_files = [
        os.path.join(image_folder, f)
        for f in sorted(os.listdir(image_folder))
        if f.lower().endswith((".png", ".jpg", ".jpeg"))
    ]

    # Iterate through the images and send them to Rerun with timestamps
    for image_file in image_files:
        # Load the image
        image = cv2.imread(image_file)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB for correct display

        # Extract timestamp from file metadata or filename
        timestamp_str = os.path.splitext(os.path.basename(image_file))[0]
        try:
            timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d_%H-%M-%S")
        except ValueError:
            timestamp = datetime.now()  # Use the current time if parsing fails

        # Send the image to Rerun with the timeline component
        rr.log_image(
            entity_path=f"image_{timestamp.strftime('%Y%m%d%H%M%S')}",
            image=image,
            timestamp=timestamp.timestamp(),
        )

if __name__ == "__main__":
    # Specify the folder containing the images
    folder_path = "/home/behnam/images/"

    # Display images with timeline
    display_images_with_timeline(folder_path)



    
