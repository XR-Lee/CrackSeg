from PIL import Image
from modeling.sync_batchnorm.replicate import patch_replication_callback
from modeling.deeplab import *
from torchvision import transforms
from dataloaders import custom_transforms as tr
import cv2
import numpy as np
import os
from utils.metrics import Evaluator
from tqdm.rich import tqdm

model = DeepLab(num_classes=2,
                        backbone='resnet',
                        output_stride=16,
                        sync_bn= None,
                        freeze_bn=False)

model = torch.nn.DataParallel(model, device_ids=[0])
patch_replication_callback(model)
model = model.cuda()

weight_name =  "/home/jc/xinrun/CrackModel/DeepLab/run/crack/deeplab-resnet/experiment_10/checkpoint11.pth.tar"
# weight_name="/home/jc/xinrun/CrackModel/DeepLab/run/crack/deeplab-resnet/finetuning_1_300ep/checkpoint300.pth.tar"
# weight_name="/home/jc/xinrun/CrackModel/DeepLab/run/crack/deeplab-resnet/experiment_8/checkpoint600.pth.tar"
# weight_name="/home/jc/xinrun/CrackModel/DeepLab/run/crack/deeplab-resnet/experiment_5/checkpoint200.pth.tar"
# weight_name="/home/jc/xinrun/CrackModel/DeepLab/run/crack/deeplab-resnet/experiment_10/checkpoint380.pth.tar"
# weight_name="/home/jc/xinrun/CrackModel/DeepLab/run/crack/deeplab-resnet/experiment_8/checkpoint.pth.tar"
# weight_name="/home/jc/Desktop/traintest/CrackModel/DeepLab/run/crack/deeplab-resnet/experiment_10/checkpoint200.pth.tar"
# checkpoint = torch.load('/home/jc/xinrun/CrackModel/DeepLab/models/20240311_checkpoint3.pth.tar')
checkpoint = torch.load(weight_name)

model.module.load_state_dict(checkpoint['state_dict'])

model.eval()
print('Model loaded')
composed_transforms = transforms.Compose([
            tr.Normalize(),
            tr.Ignore_label(),
            tr.ToTensor()])

evaluator = Evaluator(2)
print('Evaluator Loaded')




# image_folder_path = '/home/jc/dataset/20240124_0204/Images/'
# label_folder_path = '/home/jc/dataset/20240124_0204/Masks/'


# with open('/home/jc/dataset/20240124_0204/test.txt', 'r') as file:
#     filenames = [line.strip() for line in file]

image_folder_path = '/home/jc/xinrun/EvaData/Dataset/Images/'
label_folder_path = '/home/jc/xinrun/EvaData/Dataset/Masks/'
with open('/home/jc/xinrun/EvaData/Dataset/val.txt', 'r') as file:
    filenames = [line.strip() for line in file]

composed_transforms = transforms.Compose([
        tr.FixScaleCrop(crop_size=448),
        tr.Normalize(),
        tr.Ignore_label(),
        tr.ToTensor()])

evaluator.reset()
# test_images = os.listdir(image_folder_path)
for rgb_img_path in tqdm(filenames):
    # print(rgb_img_path)
    img = Image.open(image_folder_path + rgb_img_path).convert('RGB')
  

    img_label = Image.open(label_folder_path + rgb_img_path).convert('L')

    sample = {'image': img, 'label': img_label}
    sample = composed_transforms(sample)
    img = sample['image']
    target = sample['label']
    target = (target > 0).int()
    img = img.cuda()
    img = torch.unsqueeze(img, 0)
    # img = img.repeat(1, 1, 1, 1)
    with torch.no_grad():
        output = model(img)
    
    pred = output.data.cpu().numpy()
    pred = np.argmax(pred, axis=1)
    target = target.cpu().numpy()
    evaluator.add_batch(target, pred[0])
    
mIoU = evaluator.Mean_Intersection_over_Union()
F1 = evaluator.F1_Score()
Acc = evaluator.Pixel_Accuracy()
Acc_class = evaluator.Pixel_Accuracy_Class()
FWIoU = evaluator.Frequency_Weighted_Intersection_over_Union()
print("Acc:{}, Acc_class:{}, mIoU:{}, fwIoU: {}".format(Acc, Acc_class, mIoU, FWIoU))
print('F1: ', F1)
