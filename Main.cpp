
#include<Siv3D.hpp>

double dBToValue(double dB) {
	return Pow(10.0, dB / 10.0);
}


Array<double> simulate(const Array<double>& x, const std::function<double(double)>& target, double Kp, double Ki, double Kd, double y0 = 0.0, const std::function<double(double)>& disturbance = [](double) {return 0.0; }) {
	if (x.empty())
		return {};
	assert(x.isSorted());

	Array<double> y(x.size());
	y[0] = y0;
	double integral = 0.0;
	double prevDif = 0.0;
	double speed = 0.0;
	for (int j : step(x.size() - 1)) {	//for in j range(x.size() - 1):
		const double deltaT = x[j + 1] - x[j];
		assert(deltaT != 0.0);
		const double dif = target(x[j]) - y[j];
		integral += dif * deltaT;
		integral = Clamp(integral, -12.0, 12.0);

		const double p = Kp * dif;
		const double i = Ki * integral;
		const double d = j ? Kd * Clamp((dif - prevDif) / deltaT, -12.0, 12.0) : 0.0;

		speed += Clamp((p + i + d) * deltaT, -12.0, 12.0) + disturbance(x[j]) * deltaT;
		y[j + 1] = y[j] + speed * deltaT;

		prevDif = dif;
	}
	return y;
}

void Main() {
	constexpr double GUIHeight = 36;
	const double checkBoxWidth = SimpleGUI::CheckBoxRegion(U"", Vec2::Zero()).w;
	constexpr double sliderLabelWidth = 200;
	constexpr double sliderBarWidth = 300;
	const double sliderWidth = SimpleGUI::SliderRegion(Vec2::Zero(), sliderLabelWidth, sliderBarWidth).w;

	Window::SetStyle(WindowStyle::Sizable);	//手動リサイズ・最大化可
	Scene::SetScaleMode(ScaleMode::ResizeFill);	//リサイズ時シーンサイズも追従

	const auto target = [](double x)->double {	//目標値
		return 400 - 100 * 2 * (Periodic::Triangle0_1(400, x) - 0.5) + 100 * 2 * (Periodic::Square0_1(350, x) - 0.5) + 100 * 2 * (Periodic::Sine0_1(300, x) - 0.5);
	};
	const auto disturbance = [](double x)->double {	//外乱、定期的に加速度がかかる
		return std::fmod(x, 334) < 20 ? 0.01 : 0;
	};
	//ゲイン[dB]
	double Kp = -20.0;	//[dB]
	double Ki = -40.0;	//[dB]
	double Kd = -10.0;	//[dB]
	//有効フラグ(falseならゲインを0として扱う)
	bool Kpf = true, Kif = true, Kdf = true;
	constexpr double Kmin = -50;	//ゲインの最小[dB]
	constexpr double Kmax = 50;	//ゲインの最大[dB]


	//x軸
	auto x = Array<double>::IndexedGenerate(2000 * 5, [](size_t i) {return i / 5.0; });
	//y軸、シミュレート結果
	auto y = simulate(x, target, Kpf ? dBToValue(Kp) : 0.0, Kif ? dBToValue(Ki) : 0.0, Kdf ? dBToValue(Kd) : 0.0, 0.0, disturbance);

	LineString ls;
	for (int i : step(x.size()))
		ls << Vec2(x[i], y[i]);
	LineString targetLS;
	for (int i : step(x.size()))
		targetLS << Vec2(x[i], target(x[i]));

	while (System::Update()) {
		//GUI(右上固定)
		//各GUI関数は、値が更新された際にtrueを返す
		if (const double sceneWidth = Scene::Width();
			SimpleGUI::Slider(U"Kp={}dB"_fmt(Kp), Kp, Kmin, Kmax, Vec2(sceneWidth - checkBoxWidth - sliderWidth, 0), sliderLabelWidth, sliderBarWidth, Kpf)	//論理和では途中でtrueになるとそれ以降の処理がされなくなる
			| SimpleGUI::CheckBox(Kpf, U"", Vec2(sceneWidth - checkBoxWidth, 0))
			| SimpleGUI::Slider(U"Ki={}dB"_fmt(Ki), Ki, Kmin, Kmax, Vec2(sceneWidth - checkBoxWidth - sliderWidth, GUIHeight), sliderLabelWidth, sliderBarWidth, Kif)
			| SimpleGUI::CheckBox(Kif, U"", Vec2(sceneWidth - checkBoxWidth, GUIHeight))
			| SimpleGUI::Slider(U"Kd={}dB"_fmt(Kd), Kd, Kmin, Kmax, Vec2(sceneWidth - checkBoxWidth - sliderWidth, GUIHeight*2), sliderLabelWidth, 300, Kdf)
			| SimpleGUI::CheckBox(Kdf, U"", Vec2(sceneWidth - checkBoxWidth, GUIHeight*2))
			) {
			y = simulate(x, target, Kpf ? dBToValue(Kp) : 0.0, Kif ? dBToValue(Ki) : 0.0, Kdf ? dBToValue(Kd) : 0.0, 0.0, disturbance);
			ls.clear();
			for (int i : step(x.size()))
				ls << Vec2(x[i], y[i]);
		}

		targetLS.draw(5.0, Palette::Orange);	//thickness=5
		ls.draw(3.0, Palette::White);	//thickness=3
	}
}
