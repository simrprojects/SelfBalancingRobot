<?xml version='1.0' encoding='UTF-8'?>
<Project Type="Project" LVVersion="17008000">
	<Item Name="My Computer" Type="My Computer">
		<Property Name="server.app.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.control.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="server.tcp.enabled" Type="Bool">false</Property>
		<Property Name="server.tcp.port" Type="Int">0</Property>
		<Property Name="server.tcp.serviceName" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.tcp.serviceName.default" Type="Str">My Computer/VI Server</Property>
		<Property Name="server.vi.callsEnabled" Type="Bool">true</Property>
		<Property Name="server.vi.propertiesEnabled" Type="Bool">true</Property>
		<Property Name="specify.custom.address" Type="Bool">false</Property>
		<Item Name="ConvertToPosAnd3Currents.vi" Type="VI" URL="../ConvertToPosAnd3Currents.vi"/>
		<Item Name="DataParser.vi" Type="VI" URL="../DataParser.vi"/>
		<Item Name="FrameParser.vi" Type="VI" URL="../FrameParser.vi"/>
		<Item Name="main.vi" Type="VI" URL="../main.vi"/>
		<Item Name="SimpleFrameParser.vi" Type="VI" URL="../SimpleFrameParser.vi"/>
		<Item Name="SimpleFrameParserState.ctl" Type="VI" URL="../SimpleFrameParserState.ctl"/>
		<Item Name="Dependencies" Type="Dependencies">
			<Item Name="vi.lib" Type="Folder">
				<Item Name="Autoscale Polar as Needed.vi" Type="VI" URL="/&lt;vilib&gt;/picture/polarplt.llb/Autoscale Polar as Needed.vi"/>
				<Item Name="Calc Increment.vi" Type="VI" URL="/&lt;vilib&gt;/picture/scale.llb/Calc Increment.vi"/>
				<Item Name="Calc Scale Specs.vi" Type="VI" URL="/&lt;vilib&gt;/picture/scale.llb/Calc Scale Specs.vi"/>
				<Item Name="Draw Arc.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Arc.vi"/>
				<Item Name="Draw Circle by Radius.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pictutil.llb/Draw Circle by Radius.vi"/>
				<Item Name="Draw Line.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Line.vi"/>
				<Item Name="Draw Multiple Lines.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Multiple Lines.vi"/>
				<Item Name="Draw Point.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Point.vi"/>
				<Item Name="Draw Polar Grids.vi" Type="VI" URL="/&lt;vilib&gt;/picture/polarplt.llb/Draw Polar Grids.vi"/>
				<Item Name="Draw Scale.vi" Type="VI" URL="/&lt;vilib&gt;/picture/scale.llb/Draw Scale.vi"/>
				<Item Name="Draw Text at Point.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Text at Point.vi"/>
				<Item Name="Draw Text in Rect.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Draw Text in Rect.vi"/>
				<Item Name="Get Ready.vi" Type="VI" URL="/&lt;vilib&gt;/picture/polarplt.llb/Get Ready.vi"/>
				<Item Name="Get Text Rect.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Get Text Rect.vi"/>
				<Item Name="Hilite Color.vi" Type="VI" URL="/&lt;vilib&gt;/picture/pictutil.llb/Hilite Color.vi"/>
				<Item Name="Increment Filter.vi" Type="VI" URL="/&lt;vilib&gt;/picture/scale.llb/Increment Filter.vi"/>
				<Item Name="Map Setup.vi" Type="VI" URL="/&lt;vilib&gt;/picture/scale.llb/Map Setup.vi"/>
				<Item Name="Map Value to Pixel.vi" Type="VI" URL="/&lt;vilib&gt;/picture/scale.llb/Map Value to Pixel.vi"/>
				<Item Name="Move Pen.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Move Pen.vi"/>
				<Item Name="NI_AALBase.lvlib" Type="Library" URL="/&lt;vilib&gt;/Analysis/NI_AALBase.lvlib"/>
				<Item Name="Num To Text.vi" Type="VI" URL="/&lt;vilib&gt;/picture/scale.llb/Num To Text.vi"/>
				<Item Name="PCT Pad String.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/PCT Pad String.vi"/>
				<Item Name="PG angle labels.vi" Type="VI" URL="/&lt;vilib&gt;/picture/polarplt.llb/PG angle labels.vi"/>
				<Item Name="PG angle lines.vi" Type="VI" URL="/&lt;vilib&gt;/picture/polarplt.llb/PG angle lines.vi"/>
				<Item Name="PG circles.vi" Type="VI" URL="/&lt;vilib&gt;/picture/polarplt.llb/PG circles.vi"/>
				<Item Name="PG scale.vi" Type="VI" URL="/&lt;vilib&gt;/picture/polarplt.llb/PG scale.vi"/>
				<Item Name="Plot Polar Data.vi" Type="VI" URL="/&lt;vilib&gt;/picture/polarplt.llb/Plot Polar Data.vi"/>
				<Item Name="Polar Plot with Point Options.vi" Type="VI" URL="/&lt;vilib&gt;/picture/polarplt.llb/Polar Plot with Point Options.vi"/>
				<Item Name="Set Pen State.vi" Type="VI" URL="/&lt;vilib&gt;/picture/picture.llb/Set Pen State.vi"/>
				<Item Name="Validate Rectangle.vi" Type="VI" URL="/&lt;vilib&gt;/picture/scale.llb/Validate Rectangle.vi"/>
				<Item Name="VISA Configure Serial Port" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Configure Serial Port"/>
				<Item Name="VISA Configure Serial Port (Instr).vi" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Configure Serial Port (Instr).vi"/>
				<Item Name="VISA Configure Serial Port (Serial Instr).vi" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Configure Serial Port (Serial Instr).vi"/>
				<Item Name="VISA Set IO Buffer Mask.ctl" Type="VI" URL="/&lt;vilib&gt;/Instr/_visa.llb/VISA Set IO Buffer Mask.ctl"/>
			</Item>
			<Item Name="lvanlys.dll" Type="Document" URL="/&lt;resource&gt;/lvanlys.dll"/>
		</Item>
		<Item Name="Build Specifications" Type="Build"/>
	</Item>
</Project>
