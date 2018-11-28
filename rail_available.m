function rail = rail_available(vdc1,vdc2,VDC_capacity)
idx1 = find(ismember(VDC_capacity(:,1),{vdc1}),1);
idx2 = find(ismember(VDC_capacity(:,1),{vdc2}),1);
have_rail1 = VDC_capacity{idx1, 3};
have_rail2 = VDC_capacity{idx2, 3};
rail = have_rail1 && have_rail2;
end