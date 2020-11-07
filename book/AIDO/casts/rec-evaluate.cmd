sleep 1; 
echo dts challenges evaluate --challenge aido5-LF-sim-validation; 
sleep 1;     
cd /Volumes/work/DT/dt-env/dt-env-developer && DT_MOUNT=1 dts challenges evaluate \
    -C /Volumes/work/DT/dt-env/dt-env-developer/aido/challenge-aido_LF-template-random \
    --no-pull \
    --challenge aido5-LF-sim-validation; 
sleep 3;
