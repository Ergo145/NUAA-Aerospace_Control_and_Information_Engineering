function PlotPie(x, names)
h = pie(x);
hText = findobj(h,'Type','text');
percentValues = get(hText,'String');
customStrings = strcat(names,percentValues);
oldExtents_cell = get(hText,'Extent'); % cell array
oldExtents = cell2mat(oldExtents_cell); % numeric array
set(hText,{'String'},customStrings);
newExtents_cell = get(hText,'Extent'); % cell array
newExtents = cell2mat(newExtents_cell); % numeric array
width_change = newExtents(:,3)-oldExtents(:,3);
signValues = sign(oldExtents(:,1));
offset = signValues.*(width_change/2);
textPositions_cell = get(hText,{'Position'}); % cell array
textPositions = cell2mat(textPositions_cell); % numeric array
textPositions(:,1) = textPositions(:,1) + offset; % add offset adjustment
set(hText,{'Position'},num2cell(textPositions,[3,2])) % set Position property
end