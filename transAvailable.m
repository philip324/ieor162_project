function available = transAvailable(vdc1,vdc2,mode,VDC_capacity)
if mode == 'T'
    available = true;
elseif mode == 'AV'
    available = false;
elseif mode == 'R'
    VDC_cap_idx=@(s)find(cellfun(@(x)isequal(x,s), VDC_capacity(1:end,1)));
    rail1 = VDC_capacity{VDC_cap_idx(vdc1), 3};
    rail2 = VDC_capacity{VDC_cap_idx(vdc1), 3};
end