from cabang import *

# initializing the parameter values
p_diagonals = [10, 10, 10, 10, 1]
q_diagonals = [10, 10, 10, 10, 1]
p_temp, q_temp = init_pq(p_diagonals, q_diagonals)

wd = os.getcwd() #the working directory (wd) and 
pwd = os.path.dirname(wd) #the parent of the wd (pwd)

# define the max number of generations
temp_folder = pwd + "/logs/"
clear_path(temp_folder)

params = ['p', 'q']
errors_saved = ['error', 'error_bygen']
temp_names = params + ['fitness'] + errors_saved
to_load = params + ['fitness']

# defining the filename for the values temps to be saved with
path = {}
for temp in temp_names:
    path[temp] = temp_folder + temp + '_path.txt'

generasi = 20
for g in range(1, generasi+1):
    print("----------------------------------------------------------------------------------------------------")
    print("Generasi ke-"+ str(g) + " dari " + str(generasi))

    if os.path.exists(path['p']): # load the saved parameters after the 1st generation
        # backbone of GA under here
        var = load_from_path(path,to_load)
        p,q,fitness = var['p'],var['q'],var['fitness']
        #selection
        n = min(25, len(p)) #defining the max number of the individuals in the population
        
        batas_bawah=1e-03
#         daftar_parameter= [p[0],q[0]] #saving the (best?) parameters in p and q as an array
        
        #crossover (recombination of values)
        if g > 2: #picking the parents for the crossover
            indeks_ortu1,indeks_ortu2=random.sample(range(0, n), 2)
            p_ortu=p[[indeks_ortu1,indeks_ortu2],:]
            q_ortu=q[[indeks_ortu1,indeks_ortu2],:]
            fitness_ortu=fitness[[indeks_ortu1,indeks_ortu2],:]
            
            w1=fitness_ortu[0]/(fitness_ortu[0]+fitness_ortu[1])
            p, q = np.ones(25), np.ones(25)

            for j in range(25):
                pilih_ortu_p,pilih_ortu_q=np.random.random(),np.random.random()
                p[j]=(pilih_ortu_p<=w1)*p_ortu[0,j]+(pilih_ortu_p>w1)*p_ortu[1,j]
                q[j]=(pilih_ortu_q<=w1)*q_ortu[0,j]+(pilih_ortu_q>w1)*q_ortu[1,j]

        #mutation
        mp, s=0.8, 6 #mutation probability and the multiplying factor defined
        if g < 25: #the initial population number defined here
            mp,s=1,10 #for initial population, mutation is used to randomly pick the values around the inital values defines before
        penjumlah=[]
        
        apakah_mutasi=np.random.random()
        # condition of mutation existing
        if apakah_mutasi<mp: 
            for a in range(10):
                penjumlah.append((np.random.normal(0,1))) 
        # condition of mutation not existing
        else:
            penjumlah=np.zeros(10) #no addition for mutation
        
        m_p, m_q=np.zeros(25), np.zeros(25)

        i=0
        for b in range(0,25,6):
            m_p[b]=penjumlah[i]
            i+=1
        for c in range(0,25,6):
            m_q[c]=penjumlah[i]
            i+=1
            
        p_temp = (p+m_p).flatten().tolist()
        q_temp = (q+m_q).flatten().tolist()
        
        print("Matriks P generasi ke-"+str(g+1)+": "+str(np.matrix(p_temp)))
        for b in range(0,25,6):
            if p_temp[b]<batas_bawah:
                p_temp[b]=batas_bawah
                
        print("Matriks Q generasi ke-"+str(g+1)+": "+str(np.matrix(q_temp)))
        for c in range(0,25,6): #dibuat supaya q yaw tidak berubah
            if q_temp[c]<batas_bawah:
                q_temp[c]=batas_bawah
            
    #change the value of the parameter with the ones just acquired
    params_yaml(p_temp,q_temp,pwd,temp)
    p_sim,q_sim = p_temp,q_temp
    #run the launch function
    launch(pwd)
    
    # acquiring the fitness value by taking the estimation value from the temporary bag file (ga_temp.bag) and find the fitness from the error defined before
    hasil_ukf, utm, t_state, t_utm = ambil_bag(pwd)

    er_rmse, er_mae = err(hasil_ukf, utm, t_state, t_utm)
    fit = 1/er_rmse
    
    #plot the estimation for every set of parameter values acquired
    plot(hasil_ukf,utm)
    
    #saving the parameter values acquired
    catat(p_temp,q_temp,er_rmse,fit,path)
    
var = load_from_path(path,params)
p_best,q_best = var['p'][0].tolist(),var['q'][0].tolist()
params_yaml(p_best,q_best,pwd) #only works with float, not numpy.float heh