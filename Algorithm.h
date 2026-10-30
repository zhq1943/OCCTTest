#pragma once
#include "Tests.h"
class CAlgorithm {
public:
	CAlgorithm() {};
	~CAlgorithm() {};

	static void GroupBoxByLDir(std::vector<JsonWireItem>& vecWriteItems);
};

//vecWriteItems 分组功能。按照JsonBoxItem中DirX的方向分组，同向分为一组，并设置组ID
//组ID从1开始编号；非 Box 条目不含 DirX，不参与分组
void CAlgorithm::GroupBoxByLDir(std::vector<JsonWireItem>& vecWriteItems) {

	// 方向比较的角度容差（弧度）：实际数据方向向量含微小数值噪声
	const double angTol = 1.0e-6;
	std::vector<gp_Dir> groupDirs;   // 各分组的方向代表（下标 + 1 即组ID）
	int nextGroupID = 1;

	for (auto& item : vecWriteItems) {
		if (item.kind != JsonWireKind::Box)
			continue;

		JsonBoxItem& box = item.box;
		int groupID = 0;
		for (size_t g = 0; g < groupDirs.size(); ++g) {
			if (box.boxDirX.IsEqual(groupDirs[g], angTol)) {
				groupID = static_cast<int>(g) + 1;
				break;
			}
		}
		// 没有同向的已有分组，则以当前方向新建一组
		if (groupID == 0) {
			groupDirs.push_back(box.boxDirX);
			groupID = nextGroupID++;
		}
		box.groupID = groupID;
	}

}