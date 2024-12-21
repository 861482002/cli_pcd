import open3d as o3d
import numpy as np
from typing import Optional, Tuple, Union, List, Dict

def save_ply(point:Union[np.ndarray,o3d.geometry.PointCloud],out_dir:str):

    if isinstance(point,np.ndarray):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point)
        o3d.io.write_point_cloud(out_dir,pcd,write_ascii=True)
    elif isinstance(point,o3d.geometry.PointCloud):
        o3d.io.write_point_cloud(out_dir,point,write_ascii=True)


def estimate_normal(pcd:o3d.geometry.PointCloud, radius:float=0.1, max_nn:int=30):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    return pcd


if __name__ == '__main__':


    # cloud = o3d.io.read_point_cloud('./segmentaion_ply/part_0.ply')
    # radius = 0.1  # 邻域搜索半径，根据点云密度调整
    # max_nn = 30  # 搜索的最大邻域点数量
    # cloud = estimate_normal(cloud, radius, max_nn)
    # print(cloud)

    # path = './val_data_npy/clouds1_f0.npy'
    # load = np.load('./segmentaion/part1.npy')
    # print(load.shape)

    # loadtxt = np.genfromtxt('../pointcloud_data/segmented_clouds1.txt',delimiter=' ',skip_header=1)

    # for file in os.listdir('val_data'):
    #     if file.startswith('clouds1'):
    #         point_cloud = o3d.io.read_point_cloud(os.path.join('val_data', file))
    #         # o3d.visualization.draw_geometries([point_cloud])
    #         print(np.asarray(point_cloud.points).shape)
    #         # o3d.visualization.draw_geometries([point])
    #         # print(point.shape)
    
    #         # 创建可视化窗口
    #         visualizer = o3d.visualization.Visualizer()
    #         visualizer.create_window()
    
    #         # 添加点云到可视化窗口
    #         visualizer.add_geometry(point_cloud)
    
    #         # 设置渲染选项
    #         render_options = visualizer.get_render_option()
    #         render_options.point_size = 3  # 设置点的大小
    
    
    #         # 显示点云
    #         visualizer.run()
    #         visualizer.destroy_window()
    # file = mesh.Mesh.from_file('./lingjian1-1.STL')
    # print(file)
    pass