# How to compile the MPC code generation

1. Install ACADO as described in http://acado.github.io/install_linux.html

2. Source the file ACADO_ROOT/build/acdo_env.sh

3. Go into payload_mpc_controller/model

4. Prepare the make with "cmake ."

5. Compile it with "make"
   
6. Modify the parameters in "config/model.yaml" to match your model

7. Run the executable to generate all code "./quadrotor_payload_mpc_with_ext_force_codegen ../config/model.yaml" 
   
   (You can specify the model parameter file path)

Thanks to [rpg_mpc](https://github.com/uzh-rpg/rpg_mpc)
