cd acado_code_generation/;
mkdir -p build; cd build; cmake ..; make; ./main
cd ../../

for mode in "LO" "RO"; do
	rm -rf src/acado_${mode}_generated/
	cp -rf src/acado_generated_code_for_f110_nmpc/ src/acado_${mode}_generated/
	cd src/acado_${mode}_generated/
	for file in *; do
		mv $file $(echo $file | sed 's/acado_/acado_'${mode}'_/g');
		echo "ok $file"
	done

	cmd_string="%s/acado_/acado_${mode}_/ge |"
	cmd_string+="%s/ACADO_/ACADO_${mode}_/ge |"

	cmd_string+="silent %s/acadoVariables/acadoVariables_${mode}/ge |"
	cmd_string+="silent %s/acadoWorkspace/acadoWorkspace_${mode}/ge |"

	cmd_string+="wq";

	cd -

	vim -c "$cmd_string" src/acado_${mode}_generated/acado_${mode}_auxiliary_functions.c
	vim -c "$cmd_string" src/acado_${mode}_generated/acado_${mode}_auxiliary_functions.h
	vim -c "$cmd_string" src/acado_${mode}_generated/acado_${mode}_common.h
	vim -c "$cmd_string" src/acado_${mode}_generated/acado_${mode}_hessian_regularization.c
	vim -c "$cmd_string" src/acado_${mode}_generated/acado_${mode}_integrator.c
	vim -c "$cmd_string" src/acado_${mode}_generated/acado_${mode}_qpoases_interface.cpp
	vim -c "$cmd_string" src/acado_${mode}_generated/acado_${mode}_qpoases_interface.hpp
	vim -c "$cmd_string" src/acado_${mode}_generated/acado_${mode}_solver.c

	vim -c "51 s/^/#include \"acado_common.h\"" -c "wq" src/acado_${mode}_generated/acado_${mode}_common.h
	vim -c "106,307d" -c "wq" src/acado_${mode}_generated/acado_${mode}_common.h

done
