readme_amsl.txt


説明

	Graph Slamのpkg
	OpenSLAM(https://openslam.org/)の『G2O』というものを利用
	Graph構造に対して最適化処理を行うもの

使い方( [pkgName], 【NodeName】)
	[scan_match2]内の【gicp】が作成するbfr.csvが存在する場所で，以下のコマンドを実行
	$ graph_slam3d
	それにより, aft.csv(最適化処理が施されたGraph構造)が生成される.
