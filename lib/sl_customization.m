function sl_customization(cm)
% Change the order of libraries in the Simulink Library Browser. 
cm.LibraryBrowserCustomizer.applyOrder({'Flight Control Toolbox',-2});
end