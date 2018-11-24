function v_cap = get_capacity(v,VDC_capacity)
VDC_cap_idx = @(s)find(cellfun(@(x)isequal(x,s), VDC_capacity(:,1)));
v_cap = VDC_capacity{VDC_cap_idx(v),1};
end