***
# 说明
此工程用于实现ROS1下用TensorRT部署SMOKE模型实现对image图像的3D目标检测，源码及模型文件参考自 https://github.com/storrrrrrrrm/tensorrt_smoke <br>
模型部署实际效果不佳建议参考 https://github.com/lzccccc/SMOKE 工程重新训练模型
***
# 环境准备
* CUDA
* TensorRT
***
# Step1
修改CMakeLists文件以适配你的CUDA，TensorRT系统环境
***
# Step2
```bash
catkin_make
source devel/setup.bash
```
***
# Step3
修改`cfg`文件夹下`params.yaml`文件
|参数名称|功能描述|备注|
|---|---|---|
|image_topic|图像话题名称| - |
|onnx_path|onnx模型文件路径| - |
|engine_path|程序生成的engine引擎文件路径| - |
|camera\camera_width|图像宽度| - |
|camera\camera_height|图像高度| - |
|camera\camera_matrix|相机内参| - |
|camera\distort|畸变系数|当图像需要去畸变时填入，不需要时填入0.0|
***
# Step4
发布你的图像话题
***
# Step5
```bash
roslaunch camera_3d_detector run.launch #初次运行用于生成.engine文件，文件生成后需关闭程序再次运行才可进行模型推理
```
***
