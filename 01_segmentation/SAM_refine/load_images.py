import cv2
import numpy as np
from mask_process import mask_EDT
from crop_buddle import CropBunddle
class ImageLoader:

    def __init__(self, image_path, mask_path):
        self.image_path = image_path
        self.mask_path = mask_path
        self.load_raw()

    def load_raw(self):
        image = cv2.imread(self.image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mask = cv2.imread(self.mask_path,cv2.IMREAD_GRAYSCALE)
        EDT = mask_EDT(mask)
        self.raw_mask = mask.copy()
        self.raw_edt = EDT.copy()
        self.plot_image = image.copy()
        self.image = image
        self.mask = mask
        self.edt = EDT
        return  
    
    def dialate_mask(self):
        # Define a structuring element (kernel)
        kernel = np.ones((3, 3), np.uint8)
        # Apply dilation
        self.dialated_mask = cv2.dilate(self.mask, kernel, iterations=10)
        return
    
    def clustering(self):
        # Threshold the mask to identify pixels with values within the specified range
        binary_mask = cv2.inRange(self.dialated_mask, 1, 255)
        # Find connected components in the binary mask
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)
        # Create a dictionary to store the size of each cluster
        cluster_bounding_boxes = []

        # Iterate through each component
        for label in range(1, num_labels):  # Start from 1 to ignore background label
            # Get the size of the component (area)
            x, y, w, h = stats[label, cv2.CC_STAT_LEFT], stats[label, cv2.CC_STAT_TOP], \
                 stats[label, cv2.CC_STAT_WIDTH], stats[label, cv2.CC_STAT_HEIGHT]
            # Store the bounding box coordinates in the dictionary
            if w >100 or h > 100:
                cluster_bounding_boxes.append([x, y, w, h])

        ## Print the bounding box coordinates for each cluster
        # for bbox in cluster_bounding_boxes:
            # print(f"Bounding Box {bbox}")

        self.clusters = cluster_bounding_boxes
        return self.clusters

    def get_crop_bundle(self):
        if self.clusters is not None and len(self.clusters) > 0:
            crop_bundles = []
            for cluster in self.clusters:
                x, y, w, h = cluster
                # print(cluster)
                # Crop the image
                if x-200 < 0:
                    x = 200
                if y-200 < 0:
                    y = 200
                crop_frame = [y-200,y+h+200,x-200,x+w+200]
                image = self.image[crop_frame[0]:crop_frame[1], crop_frame[2]:crop_frame[3]]
                mask  =  self.mask[crop_frame[0]:crop_frame[1], crop_frame[2]:crop_frame[3]]
                EDT   =   self.edt[crop_frame[0]:crop_frame[1], crop_frame[2]:crop_frame[3]]
                crop_bundles.append( CropBunddle(image, mask, EDT,crop_frame))
            return crop_bundles
        else:
            return []
        

    def update_with_bundle(self, crop_bundle):
        crop_frame = crop_bundle.get_crop_box()
        new_mask = crop_bundle.get_mask()
        # print("bitwise_or: ",new_mask.dtype)
        # print(self.mask[crop_frame[0]:crop_frame[1], crop_frame[2]:crop_frame[3]].max())
        # print(self.mask[crop_frame[0]:crop_frame[1], crop_frame[2]:crop_frame[3]].shape)
        # print("bitwise_or: ",self.mask.dtype)
        # print(new_mask.max())
        # print(new_mask.shape)
        
        if crop_bundle.is_suc():
            self.mask[crop_frame[0]:crop_frame[1], crop_frame[2]:crop_frame[3]] = cv2.bitwise_or(
            self.mask[crop_frame[0]:crop_frame[1], crop_frame[2]:crop_frame[3]],
            new_mask
            )
            cv2.rectangle(self.raw_edt, (crop_frame[2], crop_frame[0]), (crop_frame[3], crop_frame[1]), 5, 6)
            cv2.rectangle(self.plot_image, (crop_frame[2], crop_frame[0]), (crop_frame[3], crop_frame[1]), (0, 255, 0), 6)
        else:
            cv2.rectangle(self.raw_edt, (crop_frame[2], crop_frame[0]), (crop_frame[3], crop_frame[1]), 1, 3)
            cv2.rectangle(self.plot_image, (crop_frame[2], crop_frame[0]), (crop_frame[3], crop_frame[1]), (100, 100, 100), 3)
        return

    def get_img(self):
        return self.image
    
    def get_plotimg(self):
        return self.plot_image
    
    def get_mask(self):
        return self.mask
    
    def get_rawmask(self):
        return self.raw_mask
    
    def get_rawedt(self):
        return self.raw_edt
    
    def get_edt(self):
        return self.edt
    
    def get_diatalted_mask(self):
        return self.dialated_mask