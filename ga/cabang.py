'''
this is docstring
'''
import numpy as np
import pandas as pd
import rosbag
import os
import yaml
import random
import math as m
from scipy.spatial.distance import cdist
from math import sqrt
import matplotlib.pyplot as plt
import matplotlib
import subprocess

from sklearn.metrics import mean_squared_error as mse, mean_absolute_error

def params_yaml(p,q,repo_path,temp=False):
    #read
    temp_yaml_dir = repo_path + '/config/temp.yaml'
    with open(temp_yaml_dir) as read_temp_yaml:             
        params = yaml.safe_load(read_temp_yaml)
    if q is not None:
        params['initial_estimate_covariance_P'] = p
        params['process_noise_covariance_Q'] = q

    #write and save
    if temp: #temp
        with open(temp_yaml_dir,'w') as write_temp_yaml:    
            yaml.safe_dump(params, write_temp_yaml, indent=2, allow_unicode=False) #dah berhasil ni     
    else: #best
        best_yaml_dir = repo_path + '/config/tuned.yaml'
        with open(best_yaml_dir,'w') as write_best_yaml:      
            yaml.safe_dump(params, write_best_yaml, indent=2, allow_unicode=False)
        print("Optimization is all done and the best params are now saved to {}".format(best_yaml_dir))


# function for launching the 3 nodes: rosbag play back, the ukf algorithm, the recording of the rosbag to a temporary rosbag file that saves the estimation from the UKF
# the ntoebook (with !) and python (with %?) capability to run terminal command is VERY crucial in making this tuning possible in the ROS environment. It was the one thing that was going to be the make or break.
def launch(repo_path):
    launch_path = "{}/launch/launch_for_ga.launch".format(repo_path)
    # !roslaunch {launch_path}
    os.system("roslaunch {}".format(launch_path))

    # is not str WKWKWKWK gimana ya bair keren dipakein pwd nya.
    # eh kan kalo di cmd line mala lebih gampang buat ngerun file dari folder laen?


def err(est, gt, t_est, t_gt):
    j = 1
    sample_est = []
    sample_gt = []
    for i in range(1, len(t_gt)):
        while (t_gt[i] > t_est[j]):
            j += 1
            
        sample_est.append(est[j])
        sample_gt.append(gt[i])
    rmse=sqrt(mse(sample_gt, sample_est))
    mae=mean_absolute_error(sample_gt, sample_est)
    print("rmse: "+str(rmse)+"\nmae: "+str(mae))
    return rmse,mae

#function for loading the estimation values recorded in the temporary bag file
def ambil_bag(repo_path):
    bag = rosbag.Bag(repo_path + '/bag/ga_temp.bag') #bagnya perlu udah ada ga ya? kayanya ngga, soalnya dibuatin dari launch filenya
    states_xy=[]
    states_xy_baru=[]
    t = []

    for topic, msg, time in bag.read_messages(topics=['/ukf_states']):
        states_xy.append([msg.x,msg.y])
        t.append(msg.stamp.to_sec())
    
    states_xy = np.array(states_xy)
    t = np.array(t)
    
    states_xy = states_xy[np.all(states_xy != 0,axis = 1)] #buang yg 0
    states_xy = states_xy[np.all(~np.isnan(states_xy),axis = 1)] #buang yg nan
    t = t[t != 0]

    utm=[]
    utm_baru=[]
    t_utm = []

    for topic, msg, time in bag.read_messages(topics=['/utm']):
        utm.append([msg.pose.pose.position.x,msg.pose.pose.position.y])
        t_utm.append(time.to_sec())

    utm = np.array(utm)
    t_utm = np.array(t_utm)
    min = utm[0]

    for i in range(len(states_xy)):
        pass
#         states_xy_baru.append([states_xy[i,0]-min[0],states_xy[i,1]-min[1]])
    states_xy_baru = states_xy - min #vectorization
    utm_baru = utm - min
#     states_xy_baru=np.array(states_xy_baru)
    
    for i in range(len(utm)):
        pass
#         utm_baru.append([utm[i,0]-min[0],utm[i,1]-min[1]])

    utm_baru=np.array(utm_baru)
    return states_xy_baru, utm_baru, t, t_utm


    #function for plotting the estimation and the (unextended) GNSS points 
def plot(estimasi,gnss):
    plt.figure(figsize=(10,10))
    matplotlib.style.use('default')

    plt.scatter(estimasi[:,0],estimasi[:,1],s=2.,color='k') #odometry/filtered_map
    plt.scatter(gnss[:,0],gnss[:,1],s=5.,marker="x",c='C3',alpha=1)
    plt.show(block=False) #crucial for Python script like this, as not to block the progress when the plot is shown


    # function for saving the acquired parameter values to an external .txt file
# def catat(p,q,error,fitness,p_path,q_path,error_path,error_bygen_path,
#           fitness_path):
def catat(p,q,error,fitness,path):
    p = '%9.5g' * len(p) % tuple(p)
    q = '%9.5g' * len(q) % tuple(q)
    error = str(error)
    fitness = str(fitness)

    # adding new information to the temp files
    with open(path['p'], 'a') as h:  
        h.write(p + '\n')
    with open(path['q'], 'a') as h: 
        h.write(q + '\n')
    with open(path['error'], 'a') as h:  
        h.write(error + '\n')
    with open(path['error_bygen'], 'a') as h:  
        h.write(error + '\n')
    with open(path['fitness'], 'a') as h:  
        h.write(fitness + '\n')
        
    # loading the recently modified temp files nto numpy (for easier sort ith argsort)
    x_q=np.loadtxt(path['q'], ndmin=2)  
    x_p=np.loadtxt(path['p'], ndmin=2) 
    x_error=np.loadtxt(path['error'], ndmin=2)
    x_fitness=np.loadtxt(path['fitness'], ndmin=2)
    x_fitness_f=x_fitness.flatten()
    
    # saving and sorting by its corresponding fitness value
    np.savetxt(path['p'], x_p[np.argsort(-x_fitness_f)], '%9.5g')
    np.savetxt(path['q'], x_q[np.argsort(-x_fitness_f)], '%9.5g')
    np.savetxt(path['error'], x_error[np.argsort(-x_fitness_f)], '%9.5g')
    np.savetxt(path['fitness'], x_fitness[np.argsort(-x_fitness_f)], '%9.5g')  #[np.argsortnya ini nambah dimensi]


    # delete all files in the path directory before launching
def clear_path(folder):
    os_folder = os.listdir(folder)
    if os_folder:
        for f in os_folder:
            os.remove(os.path.join(folder, f))
        print("Parameters' path is cleared!")
    else:
        print("Parameters' path is already clear!")


def init_pq(p_diag, q_diag):
    p, q = [[0,]*25]*2
    for i,j in enumerate(range(0,25,6)):
        p[j], q[j] = p_diag[i], q_diag[i]
    return p, q


def load_from_path(path,to_load):
    vars = {}
    for load in to_load:
        vars[load] = np.loadtxt(path[load], ndmin=2)
    return vars