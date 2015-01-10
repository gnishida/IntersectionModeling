#pragma once

#include "VBORenderManager.h"
#include <vector>
#include <QString>
#include <QDomNode>
#include <QVector2D>
#include "VBOBlock.h"
#include "VBOParcel.h"

class BlockSet {
public:
	std::vector<Block> blocks;
	int selectedBlockIndex;
	int selectedParcelIndex;
	//bool modified;

public:
	BlockSet() : selectedBlockIndex(-1), selectedParcelIndex(-1) {}

	//void setModified() { modified = true; }
	void load(const QString& filename);
	void save(const QString& filename);

	int selectBlock(const QVector2D& pos);
	std::pair<int, int> selectParcel(const QVector2D& pos);
	void removeSelectedBlock();

	Block& operator[](int index) { return blocks[index]; }
	Block& at(int index) { return blocks.at(index); }
	const Block& operator[](int index) const { return blocks[index]; }
	size_t size() const { return blocks.size(); }
	void clear();

private:
	void loadBlock(QDomNode& node, Block& block);
	void saveBlock(QDomDocument& doc, QDomNode& node, Block& block);
	void loadParcel(QDomNode& node, Block& block);
	void saveParcel(QDomDocument& doc, QDomNode& node, Parcel& parcel);

};

