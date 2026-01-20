#!/bin/bash

for i in {0..17}; do
    python3 -m tests.validate_aco_parameters.aco_validate_parameters "$i" &
done

wait