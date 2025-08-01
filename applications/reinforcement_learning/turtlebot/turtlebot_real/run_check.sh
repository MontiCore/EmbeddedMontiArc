. config.sh

cd "${PROJECT_ROOT}/${BINARY_GAZEBO}"
echo "Calculate the RMSE for the experiment..."
python calc_rmse.py exp1


echo "Generate a pdf file for the trajectory..."
python plot_traj.py exp1