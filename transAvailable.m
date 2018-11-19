function available = transAvailable(vdc1,vdc2,mode,VDC_capacity)
if mode == 'T'
    available = true;
elseif mode == 'R'
    VDC_cap_idx=@(s)find(cellfun(@(x)isequal(x,s), VDC_capacity(1:end,1)));
    rail1 = VDC_capacity{VDC_cap_idx(vdc1), 3};
    rail2 = VDC_capacity{VDC_cap_idx(vdc2), 3};
    if rail1 && rail2
        available = true;
    else
        available = false;
    end
else
    available = false;
end
