import os


class Path(object):
    @staticmethod
    def db_root_dir(dataset):
        if dataset == 'pascal':
            return os.environ.get('PASCAL_DATASET_ROOT', 'datasets/VOCdevkit/VOC2012/')
        if dataset == 'crack':
            return os.environ.get('CRACK_DATASET_ROOT', 'datasets/crack')
        else:
            print('Dataset {} not available.'.format(dataset))
            raise NotImplementedError
