# Grid Based DBSCAN

## Description
The main objective of this implementation is to cluster Lidar and Radar data. 3D point clouds from Lidar or Radar data is first converted into bird's eye view format and then the implementation of Grid Based DBSCAN is done.

## Data
Sample data is provided inside [data](https://github.com/arghadeep25/Grid-Based-DBSCAN/tree/master/data) folder. For additional data, the data format should be  
>   x  , y  
> 0.85,17.45  
> 0.75,15.6  
> 3.3,15.45  
> 5.25,14.2  
> 4.9,15.65  
> 5.35,15.85  


## Results

<img src="https://github.com/arghadeep25/Grid-Based-DBSCAN/blob/master/results/cluster_1_wo.png" width="200"> <img src="https://github.com/arghadeep25/Grid-Based-DBSCAN/blob/master/results/cluster_1_clusterd.png" width="200">

<img src="https://github.com/arghadeep25/Grid-Based-DBSCAN/blob/master/results/cluster_2_wo.png" width="200"> <img src="https://github.com/arghadeep25/Grid-Based-DBSCAN/blob/master/results/cluster_2_clustered.png" width="200">

<img src="https://github.com/arghadeep25/Grid-Based-DBSCAN/blob/master/results/res4.png" width="200"> <img src="https://github.com/arghadeep25/Grid-Based-DBSCAN/blob/master/results/res2.png" width="200">

## Compile
> $ git clone https://github.com/arghadeep25/Grid-Based-DBSCAN.git  
> $cd Grid-Based-DBSCAN  
> $mkdir build  
> $cd build  
> $cmake ..  
> $make  
> $./clustering ../../data/  

