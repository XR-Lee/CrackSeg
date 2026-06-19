class CropBunddle:

    def __init__(self,image,mask,edt,crop_box):
        self.image = image
        self.mask = mask
        self.edt = edt
        self.crop_box = crop_box
        self.suc_state = False

    def write_results(self, output):
        self.mask = output
        self.suc_state = True
        return
    
    def get_img(self):
        return self.image    
    
    def get_mask(self):
        return self.mask
    
    def get_edt(self):
        return self.edt
    
    def is_suc(self):
        return self.suc_state
    
    def get_crop_box(self):
        return self.crop_box