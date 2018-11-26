function v_cap = get_capacity(v,VDC_capacity)
idx = find(ismember(VDC_capacity(:,1),{v}),1);
v_cap = VDC_capacity{idx,2};
end