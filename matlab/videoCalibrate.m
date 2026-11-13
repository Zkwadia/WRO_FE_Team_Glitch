classdef HSVVideoCalibratorV1 < handle
    % HSVVIDEOCALIBRATORV1 Import a video, pause/seek, click and tune six colours.
    % Run: app = HSVVideoCalibrator;
    % Requires MATLAB R2020b or later and Image Processing Toolbox.
    % Layout adapted from the supplied ANTi_LabThresholdFinder (Atakan ERSOY).
    % Defaults and click margins follow the supplied hsv_calibrate_fast.py.
    %
    % WORKFLOW
    % Import Video -> select colour -> click upper pane -> adjust six sliders.
    % Click pauses on the currently displayed frame. Play resumes from there.
    % Seek using the timeline or seconds field. Next reads the next frame;
    % Back seeks one nominal frame interval (approximate for variable FPS).
    % Each colour retains its own calibration. All enabled colours appear
    % in the lower pane. Use Selected mask to inspect overlapping ranges.
    % Save/Load Calibration stores ALL settings in a MAT file. Export Python
    % creates a standalone HSV_RANGES dictionary; it does not edit pipelines.
    %
    % H uses 0..179; S/V use 0..255 (OpenCV-style units). H min > H max
    % means a range crossing zero. Red also has an optional 170..179 band,
    % enabled by default to match the supplied Python calibration method.
    % MATLAB HSV is scaled/rounded; OpenCV can differ by about one unit.
    % Playback processes every frame and may run slower on large videos.
    % No audio is played. Video formats depend on installed VideoReader codecs.
    properties (Access = private)
        fig; originalAxes; resultAxes; originalImage; resultImage
        sliders; fields; colourButtons; enabledBox; redBandBox
        playButton; timeline; secondsField; info; status; stats; rangeText
        viewBox; cleanupBox; areaField; opacitySlider
        reader = []; clock = []; frame = []; hsv = []; frameTime = 0
        active = 1; playing = false; closing = false; dirty = false
        names = {'RED','GREEN','MAGENTA','ORANGE','BLUE','BLACK'}
        rgb = [1 0 0;0 1 0;1 0 1;1 0.55 0;0 0.35 1;0 0 0]
        defaults = [0 10 120 255 60 255;40 90 80 255 40 200; ...
                    116 168 172 240 52 162;6 26 70 255 20 255; ...
                    94 140 45 226 58 185;0 179 0 255 0 40]
        bounds; enabled = true(1,6); redBand = true
    end
    methods
        function obj = HSVVideoCalibratorV1()
            if exist('imopen','file') ~= 2
                error('HSVVideoCalibrator:Toolbox', ...
                    'Image Processing Toolbox is required.');
            end
            obj.bounds = obj.defaults;
            obj.buildUI(); obj.syncControls();
            % Playback uses drawnow in togglePlay rather than a timer.
            % Timers are unreliable in MATLAB Online browser sessions.
        end
        function delete(obj)
            obj.closing = true;

            if ~isempty(obj.fig) && isgraphics(obj.fig)
                delete(obj.fig);
            end
        end
    end
    methods (Access = private)
        function buildUI(obj)
            screen = get(groot,'ScreenSize');
            w = min(1500,screen(3)-40); h = min(960,screen(4)-80);
            obj.fig = uifigure('Name','HSV Video Calibrator', ...
                'Position',[max(1,(screen(3)-w)/2) max(1,(screen(4)-h)/2) w h], ...
                'CloseRequestFcn',@(~,~)obj.requestClose());
            g = uigridlayout(obj.fig,[3 2]);
            g.ColumnWidth = {'1x',440}; g.RowHeight = {'1x','1x',80};
            obj.originalAxes = uiaxes(g); obj.place(obj.originalAxes,1,1);
            obj.resultAxes = uiaxes(g); obj.place(obj.resultAxes,2,1);
            for ax = [obj.originalAxes obj.resultAxes]
                ax.XTick=[]; ax.YTick=[]; ax.Interactions=[];
                ax.Toolbar.Visible='off'; disableDefaultInteractivity(ax);
            end
            title(obj.originalAxes,'Original video - click to sample and pause');
            title(obj.resultAxes,'All colours highlighted');
            panel=uipanel(g,'Title','Colour calibration','Scrollable','on');
            obj.place(panel,[1 3],2);
            c=uigridlayout(panel,[24 3],'Scrollable','on'); c.ColumnWidth={80,'1x',72};
            c.RowHeight={32,42,50,28,26,48,48,48,48,48,48,28,28,32, ...
                32,32,32,32,66,112,54,70,28,'1x'};
            obj.button(c,'Import Video',1,[1 3],@(~,~)obj.importVideo());
            obj.info=uilabel(c,'Text','No video loaded','WordWrap','on');
            obj.place(obj.info,2,[1 3]);
            colours=uigridlayout(c,[1 6]); obj.place(colours,3,[1 3]);
            colours.Padding=[0 3 0 3]; colours.ColumnSpacing=3;
            obj.colourButtons=gobjects(1,6);
            for k=1:6
                fg=[0 0 0]; if ismember(k,[1 3 5 6]), fg=[1 1 1]; end
                obj.colourButtons(k)=uibutton(colours,'Text',obj.names{k}, ...
                    'FontSize',9,'BackgroundColor',obj.rgb(k,:), ...
                    'FontColor',fg,'ButtonPushedFcn',@(~,~)obj.selectColour(k));
            end
            obj.enabledBox=uicheckbox(c,'Text','Highlight selected colour', ...
                'Value',true,'ValueChangedFcn',@(~,~)obj.changeEnabled());
            obj.place(obj.enabledBox,4,[1 3]);
            label=uilabel(c,'Text','HSV: H 0-179 | S/V 0-255 | Hue min > max wraps');
            obj.place(label,5,[1 3]);
            labels={'H min','H max','S min','S max','V min','V max'};
            obj.sliders=gobjects(1,6); obj.fields=gobjects(1,6);
            for k=1:6
                cap=255; if k<=2, cap=179; end
                label=uilabel(c,'Text',labels{k}); obj.place(label,k+5,1);
                obj.sliders(k)=uislider(c,'Limits',[0 cap], ...
                    'MajorTicks',[0 cap],'MinorTicks',[], ...
                    'ValueChangingFcn',@(~,e)obj.changeBound(k,e.Value,false), ...
                    'ValueChangedFcn',@(~,e)obj.changeBound(k,e.Value,true));
                obj.place(obj.sliders(k),k+5,2);
                obj.fields(k)=uieditfield(c,'numeric','Limits',[0 cap], ...
                    'RoundFractionalValues','on', ...
                    'ValueChangedFcn',@(s,~)obj.changeBound(k,s.Value,true));
                obj.place(obj.fields(k),k+5,3);
            end
            obj.redBandBox=uicheckbox(c,'Text','Red: also include hue 170-179', ...
                'Value',true,'ValueChangedFcn',@(~,~)obj.changeRedBand());
            obj.place(obj.redBandBox,12,[1 3]);
            obj.cleanupBox=uicheckbox(c,'Text','Clean masks: 5 x 5 opening + closing', ...
                'Value',true,'ValueChangedFcn',@(~,~)obj.settingsChanged());
            obj.place(obj.cleanupBox,13,[1 3]);
            label=uilabel(c,'Text','Min pixels'); obj.place(label,14,1);
            obj.areaField=uieditfield(c,'numeric','Limits',[0 Inf], ...
                'RoundFractionalValues','on','Value',300, ...
                'ValueChangedFcn',@(~,~)obj.settingsChanged());
            obj.place(obj.areaField,14,2);
            obj.areaField.Tooltip='Remove connected regions smaller than this pixel area. Set 0 to keep all.';
            label=uilabel(c,'Text','Overlay'); obj.place(label,15,1);
            obj.opacitySlider=uislider(c,'Limits',[0 1],'Value',0.5, ...
                'MajorTicks',[],'MinorTicks',[], ...
                'ValueChangedFcn',@(~,~)obj.settingsChanged());
            obj.place(obj.opacitySlider,15,[2 3]);
            obj.viewBox=uidropdown(c,'Items',{'All colours','Selected mask'}, ...
                'ValueChangedFcn',@(~,~)obj.render()); obj.place(obj.viewBox,16,[1 3]);
            obj.button(c,'Reset selected colour',17,[1 3],@(~,~)obj.resetColour());
            obj.button(c,'Save Calibration',18,[1 2],@(~,~)obj.saveCalibration());
            obj.button(c,'Load',18,3,@(~,~)obj.loadCalibration());
            obj.rangeText=uitextarea(c,'Editable','off'); obj.place(obj.rangeText,19,[1 3]);
            obj.stats=uitextarea(c,'Editable','off','Value',{'No video loaded'});
            obj.place(obj.stats,20,[1 3]);
            obj.button(c,'Export Python HSV_RANGES',21,[1 3],@(~,~)obj.exportPython());
            label=uilabel(c,'WordWrap','on','Text', ...
                ['Select a colour, then click the upper video to pause and sample a 5 x 5 patch. ' ...
                'Fine-tune with sliders or numbers. All six settings are retained.']);
            obj.place(label,22,[1 3]);
            obj.status=uilabel(c,'Text','Ready','WordWrap','on'); obj.place(obj.status,23,[1 3]);
            t=uigridlayout(g,[2 5]); obj.place(t,3,1);
            t.RowHeight={28,28}; t.ColumnWidth={90,70,70,'1x',110};
            obj.playButton=obj.button(t,'Play',1,1,@(~,~)obj.togglePlay());
            obj.button(t,'Back',1,2,@(~,~)obj.step(-1));
            obj.button(t,'Next',1,3,@(~,~)obj.step(1));
            label=uilabel(t,'Text','Seek to seconds:','HorizontalAlignment','right');
            obj.place(label,1,4);
            obj.secondsField=uieditfield(t,'numeric','Limits',[0 Inf], ...
                'ValueChangedFcn',@(s,~)obj.seek(s.Value)); obj.place(obj.secondsField,1,5);
            obj.timeline=uislider(t,'Limits',[0 1],'MajorTicks',[],'MinorTicks',[], ...
                'ValueChangingFcn',@(~,e)obj.scrub(e.Value), ...
                'ValueChangedFcn',@(~,e)obj.seek(e.Value));
            obj.place(obj.timeline,2,[1 5]);
        end
        function selectColour(obj,k)
            obj.active=k; obj.syncControls(); obj.render();
        end
        function syncControls(obj)
            for k=1:6
                obj.sliders(k).Value=obj.bounds(obj.active,k);
                obj.fields(k).Value=obj.bounds(obj.active,k);
                obj.colourButtons(k).Text=obj.names{k};
            end
            obj.colourButtons(obj.active).Text=['> ' obj.names{obj.active}];
            obj.enabledBox.Value=obj.enabled(obj.active);
            obj.redBandBox.Value=obj.redBand;
            obj.redBandBox.Enable='off';
            if obj.active==1, obj.redBandBox.Enable='on'; end
            b=obj.bounds(obj.active,:);
            obj.rangeText.Value={sprintf('%s  H [%d, %d]',obj.names{obj.active},b(1:2)), ...
                sprintf('S [%d, %d]   V [%d, %d]',b(3:6))};
        end
        function changeBound(obj,k,value,finished)
            obj.bounds(obj.active,k)=round(value);
            % S/V are ordered intervals; hue deliberately permits wrapping.
            if k>=3
                other=k+1; if mod(k,2)==0, other=k-1; end
                if (mod(k,2)==1 && value>obj.bounds(obj.active,other)) || ...
                   (mod(k,2)==0 && value<obj.bounds(obj.active,other))
                    obj.bounds(obj.active,other)=round(value);
                    obj.sliders(other).Value=round(value);
                    obj.fields(other).Value=round(value);
                end
            end
            obj.fields(k).Value=round(value);
            if finished, obj.syncControls(); end
            obj.dirty=true; obj.render();
        end
        function changeEnabled(obj)
            obj.enabled(obj.active)=obj.enabledBox.Value; obj.settingsChanged();
        end
        function changeRedBand(obj)
            obj.redBand=obj.redBandBox.Value; obj.settingsChanged();
        end
        function settingsChanged(obj)
            obj.dirty=true; obj.render();
        end
        function resetColour(obj)
            obj.bounds(obj.active,:)=obj.defaults(obj.active,:);
            if obj.active==1, obj.redBand=true; end
            obj.dirty=true; obj.syncControls(); obj.render();
        end
        function importVideo(obj)
            obj.pauseVideo();
            [f,p]=uigetfile({'*.mp4;*.avi;*.mov;*.m4v;*.mpg;*.mpeg;*.wmv','Video files'; ...
                '*.*','All files'},'Import video');
            if isequal(f,0), return; end
            try
                candidate=VideoReader(fullfile(p,f));
                t=candidate.CurrentTime; first=readFrame(candidate);
                obj.reader=candidate; obj.frameTime=t;
                obj.clock.Period=max(0.001,round(1000/candidate.FrameRate)/1000);
                obj.timeline.Limits=[0 max(candidate.Duration,0.001)];
                obj.info.Text=sprintf('%s | %d x %d | %.3f s | %.2f FPS', ...
                    f,candidate.Width,candidate.Height,candidate.Duration,candidate.FrameRate);
                obj.acceptFrame(first); obj.status.Text='Loaded - click a colour to calibrate';
            catch err
                uialert(obj.fig,err.message,'Cannot open video');
            end
        end
        function acceptFrame(obj,f)
            if size(f,3)==1, f=repmat(f,1,1,3); end
            obj.frame=f;
            h=rgb2hsv(f);
            h(:,:,1)=mod(round(h(:,:,1)*180),180);
            h(:,:,2:3)=round(h(:,:,2:3)*255); obj.hsv=h;
            obj.timeline.Value=min(obj.timeline.Limits(2),max(0,obj.frameTime));
            obj.secondsField.Value=obj.frameTime;
            if isempty(obj.originalImage) || ~isgraphics(obj.originalImage)
                obj.originalImage=imshow(f,'Parent',obj.originalAxes);
                obj.originalImage.ButtonDownFcn=@(~,e)obj.sample(e);
                obj.originalImage.HitTest='on'; obj.originalImage.PickableParts='all';
            else
                obj.originalImage.CData=f;
                obj.originalImage.XData=[1 size(f,2)]; obj.originalImage.YData=[1 size(f,1)];
            end
            obj.originalAxes.XLim=[0.5 size(f,2)+0.5];
            obj.originalAxes.YLim=[0.5 size(f,1)+0.5];
            title(obj.originalAxes,sprintf('Original - %.3f s - click to pause and sample',obj.frameTime));
            obj.render();
        end
        function togglePlay(obj)
            if isempty(obj.reader), return; end
            if obj.playing, obj.pauseVideo(); return; end
            if ~hasFrame(obj.reader), obj.seek(0); end
            obj.playing=true; obj.playButton.Text='Pause';
            % Keep this loop on MATLAB's UI thread. drawnow processes the
            % Pause button and click-to-sample callbacks between frames.
            nextFrameAt=tic;
            while obj.playing && isgraphics(obj.fig)
                if ~hasFrame(obj.reader)
                    obj.pauseVideo(); obj.status.Text='End of video'; break;
                end
                obj.frameTime=obj.reader.CurrentTime;
                obj.acceptFrame(readFrame(obj.reader));
                drawnow;
                wait=max(0,1/obj.reader.FrameRate-toc(nextFrameAt));
                if wait>0, pause(wait); end
                nextFrameAt=tic;
            end
        end
        function pauseVideo(obj)
            obj.playing=false;
            if isgraphics(obj.playButton)
                obj.playButton.Text='Play';
            end
        end
        function tick(obj)
            if obj.closing || ~obj.playing, return; end
            try
                if ~hasFrame(obj.reader)
                    obj.pauseVideo(); obj.status.Text='End of video'; return;
                end
                obj.frameTime=obj.reader.CurrentTime;
                obj.acceptFrame(readFrame(obj.reader));
            catch err
                obj.pauseVideo(); obj.status.Text=['Playback stopped: ' err.message];
            end
        end
        function timerError(obj)
            if ~obj.closing, obj.pauseVideo(); obj.status.Text='Playback stopped after timer error.'; end
        end
        function scrub(obj,t)
            obj.pauseVideo(); obj.secondsField.Value=t;
            obj.status.Text='Release timeline to load this timestamp';
        end
        function seek(obj,t)
            obj.pauseVideo(); if isempty(obj.reader), return; end
            % Endpoint seeks target the start of the final nominal frame.
            last=max(0,obj.reader.Duration-1/obj.reader.FrameRate);
            t=min(max(0,t),last);
            try
                obj.reader.CurrentTime=t;
                if hasFrame(obj.reader)
                    obj.frameTime=obj.reader.CurrentTime;
                    obj.acceptFrame(readFrame(obj.reader));
                    obj.status.Text='Paused - click to sample';
                end
            catch err
                uialert(obj.fig,err.message,'Seek failed');
            end
        end
        function step(obj,direction)
            obj.pauseVideo(); if isempty(obj.reader), return; end
            if direction<0
                obj.seek(obj.frameTime-1/obj.reader.FrameRate);
            elseif hasFrame(obj.reader)
                try
                    obj.frameTime=obj.reader.CurrentTime;
                    obj.acceptFrame(readFrame(obj.reader));
                catch err
                    uialert(obj.fig,err.message,'Frame read failed');
                end
            end
        end
        function sample(obj,event)
            if isempty(obj.hsv) || ~strcmp(obj.fig.SelectionType,'normal'), return; end
            obj.pauseVideo(); p=event.IntersectionPoint;
            x=round(p(1)); y=round(p(2)); [h,w,~]=size(obj.hsv);
            if x<1 || x>w || y<1 || y>h, return; end
            patch=obj.hsv(max(1,y-2):min(h,y+2),max(1,x-2):min(w,x+2),:);
            patch=reshape(patch,[],3);
            % Circular hue average prevents red (179 and 0) averaging to cyan.
            z=mean(exp(1i*patch(:,1)*2*pi/180));
            hue=mod(round(angle(z)*180/(2*pi)),180);
            sat=round(mean(patch(:,2))); val=round(mean(patch(:,3)));
            obj.bounds(obj.active,:)=[mod(hue-10,180),mod(hue+10,180), ...
                max(0,sat-60),min(255,sat+60),max(0,val-60),min(255,val+60)];
            % Neutral objects have no meaningful hue; retain all hues for black.
            if obj.active==6, obj.bounds(obj.active,1:2)=[0 179]; end
            obj.dirty=true; obj.syncControls(); obj.render();
            obj.status.Text=sprintf('Sample (%d,%d): HSV %d / %d / %d',x,y,hue,sat,val);
        end
        function render(obj)
            b=obj.bounds(obj.active,:);
            obj.rangeText.Value={sprintf('%s  H [%d, %d]',obj.names{obj.active},b(1:2)), ...
                sprintf('S [%d, %d]   V [%d, %d]',b(3:6))};
            if isempty(obj.frame), return; end
            masks=false(size(obj.hsv,1),size(obj.hsv,2),6);
            H=obj.hsv(:,:,1); S=obj.hsv(:,:,2); V=obj.hsv(:,:,3);
            se=strel('disk',2,0); counts=zeros(1,6);
            for k=1:6
                b=obj.bounds(k,:);
                if b(1)<=b(2), hm=H>=b(1) & H<=b(2);
                else, hm=H>=b(1) | H<=b(2); end
                if k==1 && obj.redBand, hm=hm | H>=170; end
                m=hm & S>=b(3) & S<=b(4) & V>=b(5) & V<=b(6);
                if obj.cleanupBox.Value, m=imclose(imopen(m,se),se); end
                if obj.areaField.Value>0, m=bwareaopen(m,obj.areaField.Value,8); end
                masks(:,:,k)=m; counts(k)=nnz(m);
            end
            if strcmp(obj.viewBox.Value,'Selected mask')
                output=repmat(uint8(masks(:,:,obj.active))*255,1,1,3);
                caption=['Mask - ' obj.names{obj.active}];
            else
                output=double(obj.frame); used=false(size(H));
                % Fixed priority prevents colours changing when switching controls.
                % All enabled masks remain independently visible in the statistics.
                for k=1:6
                    if ~obj.enabled(k), continue; end
                    m=masks(:,:,k) & ~used; used=used | m;
                    edge=bwperim(m,8); alpha=obj.opacitySlider.Value;
                    for ch=1:3
                        plane=output(:,:,ch);
                        plane(m)=(1-alpha)*plane(m)+alpha*255*obj.rgb(k,ch);
                        plane(edge)=255*obj.rgb(k,ch); output(:,:,ch)=plane;
                    end
                    % White outer rim makes black detections readable on dark video.
                    if k==6
                        rim=imdilate(m,strel('square',3)) & ~m & ~used;
                        for ch=1:3
                            plane=output(:,:,ch); plane(rim)=255; output(:,:,ch)=plane;
                        end
                    end
                end
                output=uint8(output); caption='All enabled colours - tinted fill and outlines';
            end
            if isempty(obj.resultImage) || ~isgraphics(obj.resultImage)
                obj.resultImage=imshow(output,'Parent',obj.resultAxes);
            else
                obj.resultImage.CData=output;
                obj.resultImage.XData=[1 size(output,2)]; obj.resultImage.YData=[1 size(output,1)];
            end
            obj.resultAxes.XLim=[0.5 size(output,2)+0.5];
            obj.resultAxes.YLim=[0.5 size(output,1)+0.5]; title(obj.resultAxes,caption);
            lines=cell(8,1);
            for k=1:6
                suffix=''; if ~obj.enabled(k), suffix=' (hidden)'; end
                lines{k}=sprintf('%s: %d px (%.1f%%)%s',obj.names{k},counts(k), ...
                    100*counts(k)/numel(H),suffix);
            end
            enabledMasks=masks(:,:,obj.enabled);
            overlap=nnz(sum(enabledMasks,3)>1);
            lines{7}=sprintf('Overlapping enabled pixels: %d',overlap);
            lines{8}='Overlap priority: Red > Green > Magenta > Orange > Blue > Black';
            obj.stats.Value=lines;
        end
        function saveCalibration(obj)
            obj.pauseVideo(); [f,p]=uiputfile('*.mat','Save all colours','hsv_calibration.mat');
            if isequal(f,0), return; end
            calibration=struct('version',1,'names',{obj.names},'bounds',obj.bounds, ...
                'enabled',obj.enabled,'redBand',obj.redBand, ...
                'cleanup',obj.cleanupBox.Value,'minArea',obj.areaField.Value, ...
                'opacity',obj.opacitySlider.Value,'units','H:0-179 S:0-255 V:0-255'); %#ok<NASGU>
            try
                save(fullfile(p,f),'calibration'); obj.dirty=false;
                obj.status.Text='Saved all six colour calibrations';
            catch err, uialert(obj.fig,err.message,'Save failed'); end
        end
        function loadCalibration(obj)
            obj.pauseVideo();
            if obj.dirty
                answer=uiconfirm(obj.fig,'Replace unsaved calibration settings?', ...
                    'Load calibration','Options',{'Replace','Cancel'}, ...
                    'DefaultOption',2,'CancelOption',2);
                if ~strcmp(answer,'Replace'), return; end
            end
            [f,p]=uigetfile('*.mat','Load calibration'); if isequal(f,0), return; end
            try
                data=load(fullfile(p,f),'calibration'); c=data.calibration;
                assert(isstruct(c) && isscalar(c) && c.version==1 && isequal(c.names,obj.names), ...
                    'Unsupported calibration format or colour order.');
                b=c.bounds; assert(isnumeric(b) && isequal(size(b),[6 6]) && ...
                    all(isfinite(b(:))) && all(b(:)==round(b(:))) && ...
                    all(b(:)>=0) && all(all(b(:,1:2)<=179)) && ...
                    all(all(b(:,3:6)<=255)) && all(b(:,3)<=b(:,4)) && ...
                    all(b(:,5)<=b(:,6)),'Invalid HSV bounds.');
                assert(islogical(c.enabled) && isequal(size(c.enabled),[1 6]) && ...
                    islogical(c.redBand) && isscalar(c.redBand) && ...
                    islogical(c.cleanup) && isscalar(c.cleanup),'Invalid mask settings.');
                validateattributes(c.minArea,{'numeric'},{'scalar','finite','integer','nonnegative'});
                validateattributes(c.opacity,{'numeric'},{'scalar','finite','>=',0,'<=',1});
                obj.bounds=b; obj.enabled=c.enabled; obj.redBand=c.redBand;
                obj.cleanupBox.Value=c.cleanup; obj.areaField.Value=c.minArea;
                obj.opacitySlider.Value=c.opacity; obj.dirty=false;
                obj.syncControls(); obj.render(); obj.status.Text='Loaded all six colours';
            catch err, uialert(obj.fig,err.message,'Invalid calibration file'); end
        end
        function exportPython(obj)
            obj.pauseVideo(); [f,p]=uiputfile('*.py','Export thresholds','hsv_ranges.py');
            if isequal(f,0), return; end
            try
                [fid,msg]=fopen(fullfile(p,f),'w'); if fid<0, error('%s',msg); end
                cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
                fprintf(fid,'import numpy as np\n\n# OpenCV HSV units. Inclusive bounds.\n');
                fprintf(fid,'# For wrapped colours, OR all numbered ranges.\nHSV_RANGES = {\n');
                for k=1:6
                    b=obj.bounds(k,:); bands=[b(1) b(2)];
                    if b(1)>b(2), bands=[0 b(2);b(1) 179]; end
                    if k==1 && obj.redBand
                        bands=[bands;170 179]; %#ok<AGROW>
                        bands=sortrows(bands,1); merged=bands(1,:);
                        for j=2:size(bands,1)
                            if bands(j,1)<=merged(end,2)+1
                                merged(end,2)=max(merged(end,2),bands(j,2));
                            else, merged=[merged;bands(j,:)]; end %#ok<AGROW>
                        end
                        bands=merged;
                    end
                    % Keep RED_1 and RED_2 keys even when only one band remains.
                    if k==1 && size(bands,1)==1, bands=[bands;bands]; end
                    for j=1:size(bands,1)
                        key=obj.names{k};
                        if size(bands,1)>1, key=sprintf('%s_%d',key,j); end
                        fprintf(fid,'    "LOWER_%s": np.array([%d, %d, %d], dtype=np.uint8),\n', ...
                            key,bands(j,1),b(3),b(5));
                        fprintf(fid,'    "UPPER_%s": np.array([%d, %d, %d], dtype=np.uint8),\n', ...
                            key,bands(j,2),b(4),b(6));
                    end
                end
                fprintf(fid,'}\n\n# Preview settings (apply separately in your pipeline).\n');
                fprintf(fid,'MIN_REGION_PIXELS = %d\n',obj.areaField.Value);
                flags={'False','True'};
                fprintf(fid,'CLEAN_MASKS = %s\n',flags{double(obj.cleanupBox.Value)+1});
                fprintf(fid,'ENABLED_COLOURS = [');
                for k=find(obj.enabled), fprintf(fid,'"%s", ',obj.names{k}); end
                fprintf(fid,']\n');
                [writeMessage,writeError]=ferror(fid);
                if writeError~=0, error('%s',writeMessage); end
                obj.status.Text='Exported Python ranges';
            catch err, uialert(obj.fig,err.message,'Export failed'); end
        end
        function requestClose(obj)
            obj.pauseVideo();
            if obj.dirty
                answer=uiconfirm(obj.fig,'Save your calibration before closing?', ...
                    'Unsaved calibration','Options',{'Save','Discard','Cancel'}, ...
                    'DefaultOption',1,'CancelOption',3);
                if strcmp(answer,'Cancel'), return; end
                if strcmp(answer,'Save')
                    obj.saveCalibration(); if obj.dirty, return; end
                end
            end
            delete(obj);
        end
    end
    methods (Static, Access = private)
        function place(control,row,column)
            control.Layout.Row=row; control.Layout.Column=column;
        end
        function b=button(parent,text,row,column,callback)
            b=uibutton(parent,'Text',text,'ButtonPushedFcn',callback);
            HSVVideoCalibratorV1.place(b,row,column);
        end
    end
end
