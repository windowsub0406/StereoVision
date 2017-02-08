# Stereo Vision


<p align="center">
    <img src="images/picture1.png" width="480" alt="bm_wls_image" /><br>
</p>
  
  
## Introduction  
  
  
>This project was coded in 2015 for self-driving project using stereo vision. I had conducted self-driving based on stereo vision. This code doesn't include detection & tracking. In this code, My point was preprocessing for exact obstacle detection.


## Summary  
  
  
* I used **Block Matching**(BM) Algorithm for a disparity map. 
* I separated ground and unnecessary parts(above the height of a vehicle) by using **v-disparity** method.
* I coded **occupancy grid** which is one of General Object Detection(GOD) methods.
* I tried **stixels** method.


## Installation & Environment  
 
### Installation
 
* __Dataset__  
 
    [Daimler dataset](
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    dfasdfasdfa!!!!!!!!!!!!!!!https://d17h27t6h515a5.cloudfront.net/topher/2016/December/584f6edd_data/data.zip) (Track1 data provided by Udacity)
 
 
### Environment  
  
#### software  
  
>Windows 10(x64), tensorflow 0.12.1, keras, Python 3.5, OpenCV 3.1.0, pycharm  
 
#### hardware  
  
>CPU : i7-4720HQ 2.60Hz, GPU : GTX 970M, Memory : 16GB