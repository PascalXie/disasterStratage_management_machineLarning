cd alice_step3_dynamicDeployment_version2/build/

rm -r *
cmake ..
make

for i in {1..10}
do
	echo Run ID: $i
	./alice
done

cd ../../
