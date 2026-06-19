import matplotlib.pyplot as plt



def show_img_in_jupyter(img, title=None):
    plt.figure(figsize=(3,3))
    plt.imshow(img, cmap='viridis')
    plt.axis('on')
    plt.colorbar()
    if title:
        plt.title(title)
    plt.show()
