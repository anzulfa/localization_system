in the process of tidying up and transforming the jupyter ntoebook into a clean, neat, modular set of pys

### to dos: 

#### tidying up with modularity to pys 
- [x] py script fungsional
- supporting:
- [x] pindah ke path yg bener dan semua files pengikutnya disesuaiin (LAUNCH gimana ya) 
- [ ] numpy yg penting cuman buat arg sort ga si (cuman di fungsi ambil bag berarti?). yg lain ganti list
- [x] apus semua path file di folder d di awal launch
- [x] coba pengganti ! (%?) buat di py script -- os.system("terminal command")
- [x] masukin readme
- [ ] semua yg panjang2 di main (termasuk 3 operator) jadi fungsi masing2 (class ga)
- [x] parameter global harus dioper jadi argumen tiap fungsi lain (untuk mendukung saat dijadikan moduler)


#### improvement to the algorithms
- [x] output tuned.yaml di akhir proses loop (baik setelah akhir generasi atau callback)
- [ ] plot realtime progres eror di to do (berat)
- [ ] make the parameters vars customized (berat)
- [ ] early stopping callback
- [ ] scaling of the mutation increments
- [ ] buat pilihan jenis **mutasi** dan **persilangan**

### the result
The parameters obtained from the GA tuning process can be found as a YAML file named tuned.yaml in the config folder. To utilize these, you may modify the params of any of the launch file and change the rosparam line to take the tuned.yaml file like so:


![rosparam dark](https://user-images.githubusercontent.com/67263982/115954070-1cbf5a00-a519-11eb-858e-427c511f5e34.png)
