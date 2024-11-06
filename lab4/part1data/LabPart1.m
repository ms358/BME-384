clc
clear 
close all
[XpA, YpA, TzA, meanXpA, meanYpA, stdXpA, stdYpA]= parseNplot("Part A","Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv",'Group1_BME384_partA_NIDAQ_PCI-6221_22859738.csv',10001,10);
[XpB, YpB, TzB, meanXpB, meanYpB, stdXpB, stdYpB]= parseNplot("Part B", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partB_NIDAQ_PCI-6221_22859738.csv",10001,10);
[XpC1, YpC1, TzC1, meanXpC1, meanYpC1, stdXpC1, stdYpC1]= parseNplot("Part C1", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partC1_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpC2, YpC2, TzC2, meanXpC2, meanYpC2, stdXpC2, stdYpC2]= parseNplot("Part C2", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partC2_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpC3, YpC3, TzC3, meanXpC3, meanYpC3, stdXpC3, stdYpC3]= parseNplot("Part C3", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partC3_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpC4, YpC4, TzC4, meanXpC4, meanYpC4, stdXpC4, stdYpC4]= parseNplot("Part C4", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partC4_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpC5, YpC5, TzC5, meanXpC5, meanYpC5, stdXpC5, stdYpC5]= parseNplot("Part C5", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partC5_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpD1, YpD1, TzD1, meanXpD1, meanYpD1, stdXpD1, stdYpD1]= parseNplot("Part D1", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partD1_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpD2, YpD2, TzD2, meanXpD2, meanYpD2, stdXpD2, stdYpD2]= parseNplot("Part D2", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partD2_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpD3, YpD3, TzD3, meanXpD3, meanYpD3, stdXpD3, stdYpD3]= parseNplot("Part D3", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partD3_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpD4, YpD4, TzD4, meanXpD4, meanYpD4, stdXpD4, stdYpD4]= parseNplot("Part D4", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partD4_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpD5, YpD5, TzD5, meanXpD5, meanYpD5, stdXpD5, stdYpD5]= parseNplot("Part D5", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partD5_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpE1, YpE1, TzE1, meanXpE1, meanYpE1, stdXpE1, stdYpE1]= parseNplot("Part E1", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partE1_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpE2, YpE2, TzE2, meanXpE2, meanYpE2, stdXpE2, stdYpE2]= parseNplot("Part E2", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partE2_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpE3, YpE3, TzE3, meanXpE3, meanYpE3, stdXpE3, stdYpE3]= parseNplot("Part E3", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partE3_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpE4, YpE4, TzE4, meanXpE4, meanYpE4, stdXpE4, stdYpE4]= parseNplot("Part E4", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partE4_NIDAQ_PCI-6221_22859738.csv",6001,6);
[XpE5, YpE5, TzE5, meanXpE5, meanYpE5, stdXpE5, stdYpE5]= parseNplot("Part E5", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partE5_NIDAQ_PCI-6221_22859738.csv",6001,6);
%% Optitrack data
idle("Group1_BME384_partA.csv", 1000, XpA, YpA, TzA, stdXpA, stdYpA, meanXpA, meanYpA, 10001)
idle("Group1_BME384_partB.csv", 1000, XpB, YpB, TzB, stdXpB, stdYpB, meanXpB, meanYpB, 10001)
idle("Group1_BME384_partC1.csv", 600, XpC1, YpC1, TzC1, stdXpC1, stdYpC1, meanXpC1, meanYpC1, 6001)
idle("Group1_BME384_partC2.csv", 600, XpC2, YpC2, TzC2, stdXpC2, stdYpC2, meanXpC2, meanYpC2, 6001)
idle("Group1_BME384_partC3.csv", 600, XpC3, YpC3, TzC3, stdXpC3, stdYpC3, meanXpC3, meanYpC3, 6001)
idle("Group1_BME384_partC4.csv", 600, XpC4, YpC4, TzC4, stdXpC4, stdYpC4, meanXpC4, meanYpC4, 6001)
idle("Group1_BME384_partC5.csv", 600, XpC5, YpC5, TzC5, stdXpC5, stdYpC5, meanXpC5, meanYpC5, 6001)
idle("Group1_BME384_partD1.csv", 600, XpD1, YpD1, TzD1, stdXpD1, stdYpD1, meanXpD1, meanYpD1, 6001)
idle("Group1_BME384_partD2.csv", 600, XpD2, YpD2, TzD2, stdXpD2, stdYpD2, meanXpD2, meanYpD2, 6001)
idle("Group1_BME384_partD3.csv", 600, XpD3, YpD3, TzD3, stdXpD3, stdYpD3, meanXpD3, meanYpD3, 6001)
idle("Group1_BME384_partD4.csv", 600, XpD4, YpD4, TzD4, stdXpD4, stdYpD4, meanXpD4, meanYpD4, 6001)
idle("Group1_BME384_partD5.csv", 600, XpD5, YpD5, TzD5, stdXpD5, stdYpD5, meanXpD5, meanYpD5, 6001)
idle("Group1_BME384_partE1.csv", 600, XpE1, YpE1, TzE1, stdXpE1, stdYpE1, meanXpE1, meanYpE1, 6001)
idle("Group1_BME384_partE2.csv", 600, XpE2, YpE2, TzE2, stdXpE2, stdYpE2, meanXpE2, meanYpE2, 6001)
idle("Group1_BME384_partE3.csv", 600, XpE3, YpE3, TzE3, stdXpE3, stdYpE3, meanXpE3, meanYpE3, 6001)
idle("Group1_BME384_partE4.csv", 600, XpE4, YpE4, TzE4, stdXpE4, stdYpE4, meanXpE4, meanYpE4, 6001)
idle("Group1_BME384_partE5.csv", 600, XpE5, YpE5, TzE5, stdXpE5, stdYpE5, meanXpE5, meanYpE5, 6001)