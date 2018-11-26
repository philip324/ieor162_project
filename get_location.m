function loc = get_location(p,location)
% lst1 = all(ismember(VDCs,path(j)),2);
% find(lst1)
VDC_loc_idx = @(s)find(cellfun(@(x)isequal(x,s), location(:,1)));
dealer_loc_idx = @(i) i-1;
if isa(p,'char')
    lat = location{VDC_loc_idx(p), 3};
    long = location{VDC_loc_idx(p), 4};
    loc = [lat, long];
elseif isa(p,'double')
    lat = location{dealer_loc_idx(p), 3};
    long = location{dealer_loc_idx(p), 4};
    loc = [lat, long];
end
end