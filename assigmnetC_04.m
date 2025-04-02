
dataset_4GHz_single = ['-10', '10', '30', '50', '70'];
dataset_1GHz_single = ['-10_1GHz', '10_1GHz', '30_1GHz', '50_1GHz', '70_1GHz'];
dataset_4GHz_double = ['-1010', '1030', '3050', '5070'];
dataset_1GHz_double = ['-1010_1GHz', '1030_1GHz', '3050_1GHz', '5070_1GHz'];

dataset = dataset_4GHz_single
for i = 1:length(dataset)
    readTIRawData_CCS_capture_demo_PROCESS_2016a_BETA
end