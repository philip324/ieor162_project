function rail = rail_available(vdc1,vdc2,VDC_capacity)
VDC_cap_idx = @(s)find(cellfun(@(x)isequal(x,s), VDC_capacity(:,1)));
have_rail1 = VDC_capacity{VDC_cap_idx(vdc1), 3};
have_rail2 = VDC_capacity{VDC_cap_idx(vdc2), 3};
rail = have_rail1 && have_rail2;
end