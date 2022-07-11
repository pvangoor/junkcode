n_runs=10000
output0=""
for ((i=0;i<$n_runs;++i)); do
    output1=$(./random_ints)
    if [ "$output0" != "$output1" ]; then
        echo "RUN $i: $output1"
        output0=$output1
    fi
done