#include "file_interface.h"

namespace SlamCommon
{

CFileInterface::CFileInterface(const std::string &cur_path, CConfigFileOperator *config_operator)
	: m_cur_path(cur_path), m_config_oper(config_operator), vel_(0.0), vel_y_(0.0), vel_w_(0.0), score_(0.0), 
    autorunState_(false), match_mode_(1), set_submap2d_protos_finished_(false), ref_cols_position_x_(0.0),
    ref_cols_position_y_(0.0)
{
    ref_cols_response_results_ = "";
}

CFileInterface::~CFileInterface()
{
}

/*********************************************************************
function: save map data
*********************************************************************/
void CFileInterface::SaveMapData(const std::string& map_name,
	const OccupancyGrid &occupancy_grid)
{
    const std::string map_path = GetDocsPath() + DOCS_MAP_ID + DIR_FLAG;
	//save key parameters to xml file
    const std::string xml_name(map_path + map_name + XML_ID);
	WriteXml(map_name, xml_name, occupancy_grid);

	//save map data to pgm file
    const std::string pgm_name(map_path + map_name + PGM_ID);
    WritePgm(map_name, pgm_name, occupancy_grid);
//    WritePgmWithFile(map_name, pgm_name, occupancy_grid);
}

//penghu 24/5/09
void CFileInterface::SaveExpandMapData(const std::string& map_name,
    const OccupancyGrid &occupancy_grid)
{
    const std::string map_path = GetDocsPath() + DOCS_EXPAND_MAP_ID + DIR_FLAG;
    //save key parameters to xml file
    const std::string xml_name(map_path + map_name + XML_ID);
    WriteXml(map_name, xml_name, occupancy_grid);

    //save map data to pgm file
    const std::string pgm_name(map_path + map_name + PGM_ID);
    //LOG_INFO("yuntai_start get pgm4");
    WritePgm(map_name, pgm_name, occupancy_grid);
//    WritePgmWithFile(map_name, pgm_name, occupancy_grid);
}

void CFileInterface::WritePgmWithFile(const std::string &pgm_name, const OccupancyGrid &occupancy_grid)
{
    // LOG_INFO("Writing map occupancy data to %s", pgm_name.c_str());
    FILE* out = fopen(pgm_name.c_str(), "w");
    if (!out)
    {
        // LOG_ERROR("Couldn't save map file to %s", pgm_name.c_str());
        return;
    }
    fprintf(out, "P5\n# CSG map, %.3f m/pix\n%d %d\n255\n",
            occupancy_grid.resolution, occupancy_grid.width, occupancy_grid.height);
    for(unsigned int y = 0; y < occupancy_grid.height; y++)
    {
        for(unsigned int x = 0; x < occupancy_grid.width; x++)
        {
            unsigned int i = x + (occupancy_grid.height - y - 1) * occupancy_grid.width;
            if (occupancy_grid.cells[i] >= 0 && occupancy_grid.cells[i] <= 100)
            {
//                int a = (int)((100 - occupancy_grid.cells[i]) * 255 / 100 + 0.5);
//                fprintf(out, "%d ", 255);
                fputc((int)((100 - occupancy_grid.cells[i]) * 255 / 100 + 0.5), out);
            }
            else
            {
//                fprintf(out, "%d ", UNKNOWN_VALUE);
                fputc(UNKNOWN_VALUE, out);
            }
        }
    }
    fclose(out);
}

/*********************************************************************
function: write pgm file
*********************************************************************/
void CFileInterface::WritePgm(const std::string &map_name,
	const std::string &pgm_name, const OccupancyGrid &occupancy_grid)
{
    // LOG_INFO("Saving map to %s...", pgm_name.c_str());
    std::ofstream pgm_file(pgm_name, std::ios::out | std::ios::binary);

	//PGM header
	const std::string header = "P5\n# CSG map: " + map_name + ", resolution: "
        + NumToString(occupancy_grid.resolution) + " m/pixel\n"
		+ std::to_string(occupancy_grid.width) + " "
		+ std::to_string(occupancy_grid.height) + "\n255\n";
	m_config_oper->WriteStringToFile(pgm_file, header);

	// We choose a value between the free and occupied threshold.

	for (size_t y = 0; y < occupancy_grid.height; ++y)
	{
		for (size_t x = 0; x < occupancy_grid.width; ++x)
		{
			const size_t i = x + (occupancy_grid.height - y - 1) * occupancy_grid.width;

			//transform the probability([0, 100]) of grid map to gray value[0, 255]
			//the higher of the probability, closer to 0, 0 means black, 255 means wight
            if (occupancy_grid.cells[i] >= 0 && occupancy_grid.cells[i] <= 100)
            {
                int tmpt = (int)((100 - occupancy_grid.cells[i]) * 255 / 100 + 0.5);
				//shade map
// 				m_config_oper->WriteCharToFile(pgm_file, tmpt);
				//two value map
				if(tmpt < UNKNOWN_VALUE)
				{
					m_config_oper->WriteCharToFile(pgm_file, 0);
				}
				else
				{
					m_config_oper->WriteCharToFile(pgm_file, 255);
				}
            }
            else
            {
                m_config_oper->WriteCharToFile(pgm_file, UNKNOWN_VALUE);
            }
		}
	}
	pgm_file.close();
}

/*********************************************************************
function: write xml file
*********************************************************************/
void CFileInterface::WriteXml(const std::string &map_name,
	const std::string& xml_name, const OccupancyGrid &occupancy_grid)
{
    // LOG_INFO("Saving map to %s..", xml_name.c_str());
    std::ofstream xml_file(xml_name, std::ios::out | std::ios::binary);

	m_config_oper->InitConfigFileOperatorForWriting();

	m_config_oper->SetParam("image", map_name);
    m_config_oper->SetParam("resolution", NumToString(occupancy_grid.resolution, 3));
	//origin in current map
    m_config_oper->SetParam("origin_pose_x", NumToString(occupancy_grid.origin.x, 4));
    m_config_oper->SetParam("origin_pose_y", NumToString(occupancy_grid.origin.y, 4));
    m_config_oper->SetParam("origin_pose_theta", NumToString(occupancy_grid.origin.theta, 4));

    m_config_oper->SetParam("occupied_threash", NumToString(occupancy_grid.occupied_threash));
    m_config_oper->SetParam("free_threash", NumToString(occupancy_grid.free_threash));
	m_config_oper->SetParam("negate", occupancy_grid.negate);

	m_config_oper->WriteXmlFile(xml_name);//write to xml
	m_config_oper->UninitConfigFileOperator();

	xml_file.close();
//     CHECK(xml_file) << "Writing " << xml_name << " failed.");
}

/*********************************************************************
function: the interface for getting map data, and need be destroyed by
	the user; if can't find map name, return nullptr
*********************************************************************/
OccupancyGrid *CFileInterface::GetOccupancyGrid(const std::string &map_name, const bool read_pgm)
{
	OccupancyGrid *occupancy_grid = new OccupancyGrid;

	if (read_pgm && !ReadMapFromPgm(map_name, occupancy_grid))
	{
		// LOG_ERROR("Read map pgm data failed!");
		return nullptr;
	}

	if (!ReadMapFromXml(map_name, occupancy_grid))
	{
		// LOG_ERROR("Read map xml data failed!");
		return nullptr;
	}

	return occupancy_grid;
}

/*********************************************************************
function: read map data from pgm file
*********************************************************************/
bool CFileInterface::ReadMapFromPgm(const std::string &map_name, OccupancyGrid *occupancy_grid)
{
	std::string str_tempt;
	std::stringstream str_stream;
    std::string map_path = GetDocsPath()
		+ DOCS_MAP_ID + DIR_FLAG + map_name + PGM_ID;
	std::ifstream infile;

	infile.open(map_path.c_str());

	if (!infile.is_open())
	{
        // LOG_ERROR("Open %s failed!", map_path.c_str());
		return false;
	}

	int count = 0;
    int y = 0, x = 0;
	bool is_empty = false;
	while (infile.good() && !infile.eof())
	{
		str_stream.clear();
		str_stream.str("");
        getline(infile, str_tempt);
		str_stream << str_tempt;
		switch (count++)
		{
			//line 1: P5
		case 0:
			break;
			//line 2: name and resolution
        case 1:
			break;
			//line 3: width and height
		case 2:
            str_stream >> occupancy_grid->width >> occupancy_grid->height;
// 			LOG_INFO("width = " << occupancy_grid->width << ", height = " << occupancy_grid->height;
			break;
			//line 4:  255
		case 3:
			break;
			//the following lines: pgm data
		case 4:
			//only do once
			y = occupancy_grid->height - 1;
			occupancy_grid->cells.resize(occupancy_grid->width *  occupancy_grid->height);
		default:
		{
			size_t index = 0;
// 			LOG_INFO("begin y = " << y << ", x = " << x;
			//the format of the data read from pgm is char
			unsigned char c;
			for (; y >= 0; --y)
			{
				if (!is_empty)
				{
					x = 0;
				}
				for (; x < (int)occupancy_grid->width; ++x)
				{
					is_empty = false;
					index = x + y * occupancy_grid->width;
					if(str_stream >> c)
					{
//                        printf("x = %d, y = %d, i = %d, c = %d\n", x, y , index, c);
						//unknown is -1, others are in [0, 100]
						if (UNKNOWN_VALUE == c)
						{
							occupancy_grid->cells[index] = -1;
						}
						else
						{
							occupancy_grid->cells[index] = 100 - (int)(c * 100 / 255 + 0.5);
						}
						continue;
                    }
					is_empty = true;
					break;
				}
				if (is_empty)
				{
					break;
				}
			}
			break;
		}
		}
	}
	infile.close();
//    LOG_INFO("Read %s successfully!", map_path.c_str());
    str_stream.clear();
    str_stream.str("");
	return true;
}

//penghu 24/5/09
OccupancyGrid *CFileInterface::GetExpandGrid(const std::string &map_name)
{
    OccupancyGrid *occupancy_grid = new OccupancyGrid;

    if (!ReadExpandMapFromPgm(map_name, occupancy_grid))
    {
        // LOG_ERROR("Read map pgm data failed!");
        return nullptr;
    }

    return occupancy_grid;
}

//penghu 24/5/09
bool CFileInterface::ReadExpandMapFromPgm(const std::string &map_name, OccupancyGrid *occupancy_grid)
{
    std::string str_tempt;
    std::stringstream str_stream;
    std::string map_path = GetDocsPath()
        + DOCS_EXPAND_MAP_ID + DIR_FLAG + map_name + PGM_ID;
    std::ifstream infile;

    infile.open(map_path.c_str());

    if (!infile.is_open())
    {
        // LOG_ERROR("Open %s failed!", map_path.c_str());
        return false;
    }

    int count = 0;
    int y = 0, x = 0;
    bool is_empty = false;
    while (infile.good() && !infile.eof())
    {
        str_stream.clear();
        str_stream.str("");
        getline(infile, str_tempt);
        str_stream << str_tempt;
        switch (count++)
        {
            //line 1: P5
        case 0:
            break;
            //line 2: name and resolution
        case 1:
            break;
            //line 3: width and height
        case 2:
            str_stream >> occupancy_grid->width >> occupancy_grid->height;
// 			LOG_INFO("width = " << occupancy_grid->width << ", height = " << occupancy_grid->height;
            break;
            //line 4:  255
        case 3:
            break;
            //the following lines: pgm data
        case 4:
            //only do once
            y = occupancy_grid->height - 1;
            occupancy_grid->cells.resize(occupancy_grid->width *  occupancy_grid->height);
        default:
        {
            size_t index = 0;
// 			LOG_INFO("begin y = " << y << ", x = " << x;
            //the format of the data read from pgm is char
            unsigned char c;
            for (; y >= 0; --y)
            {
                if (!is_empty)
                {
                    x = 0;
                }
                for (; x < (int)occupancy_grid->width; ++x)
                {
                    is_empty = false;
                    index = x + y * occupancy_grid->width;
                    if(str_stream >> c)
                    {
//                        printf("x = %d, y = %d, i = %d, c = %d\n", x, y , index, c);
                        //unknown is -1, others are in [0, 100]
                        if (UNKNOWN_VALUE == c)
                        {
                            occupancy_grid->cells[index] = -1;
                        }
                        else
                        {
                            occupancy_grid->cells[index] = 100 - (int)(c * 100 / 255 + 0.5);
                        }
                        continue;
                    }
                    is_empty = true;
                    break;
                }
                if (is_empty)
                {
                    break;
                }
            }
            break;
        }
        }
    }
    infile.close();
//    LOG_INFO("Read %s successfully!", map_path.c_str());
    str_stream.clear();
    str_stream.str("");
    return true;
}

/*********************************************************************
function: read map parameters from xml
*********************************************************************/
bool CFileInterface::ReadMapFromXml(const std::string &map_name, 
	OccupancyGrid *occupancy_grid)
{
    m_config_oper->SetNodeDirectory(DIR_DOCS_ID + std::string(DIR_FLAG) + DOCS_MAP_ID);
	if (!m_config_oper->InitConfigFileOperator(map_name))
	{
		return false;
	}

	m_config_oper->GetParam("image", occupancy_grid->map_name, std::string("map_0"));
	//insure the reading map is which we want
	if (map_name != occupancy_grid->map_name)
	{
		return false;
	}
	m_config_oper->GetParam("resolution", occupancy_grid->resolution, 0.03);

	Pose3D pose;
	m_config_oper->GetParam("origin_pose_x", pose.x, 0.0);
	m_config_oper->GetParam("origin_pose_y", pose.y, 0.0);
	m_config_oper->GetParam("origin_pose_theta", pose.theta, 0.0);
	occupancy_grid->origin = pose;

	m_config_oper->GetParam("occupied_threash", occupancy_grid->occupied_threash, 0.51);
	m_config_oper->GetParam("free_threash", occupancy_grid->free_threash, 0.49);
	m_config_oper->GetParam("negate", occupancy_grid->negate, false);

	m_config_oper->UninitConfigFileOperator();
//    LOG_INFO("Read %s successfully!", (map_name + XML_ID).c_str());
	return true;
}

/*********************************************************************
function: save trajectory
*********************************************************************/
void CFileInterface::SaveTrajectoryToTxt(const std::string traj_name, const ReflectorMapData &traj)
{
    //save as txt
    const std::string txt_name = GetDocsPath()
            + DOCS_MAPPING_TRAJECTORY_ID + DIR_FLAG + traj_name + TXT_ID;
    std::ofstream traj_file(txt_name, std::ios::out | std::ios::binary);

	size_t size = traj.pose.size();
    std::string line(traj_name + "\n");
	//write map name relative with trajectory into file
	m_config_oper->WriteStringToFile(traj_file, line);
	//write trajectory points into file
	for (size_t i = 0; i < size; ++i)
	{
        line = NumToString(traj.pose[i].x) + " "
			 + NumToString(traj.pose[i].y) + " "
             + NumToString(traj.pose[i].theta) + "\n";
		m_config_oper->WriteStringToFile(traj_file, line);
	}
	traj_file.close();
//     CHECK(traj_file) << "Writing " << txt_name << " failed.");
    // LOG_INFO("Save %s successfully!", txt_name.c_str());
}

/*********************************************************************
function: read trajectory from txt file
*********************************************************************/
bool CFileInterface::ReadTrajectoryFromTxt(const std::string &traj_name, ReflectorMapData &traj)
{
	std::string str_tempt;
	std::stringstream ss;
    std::string traj_path = GetDocsPath() + DOCS_MAPPING_TRAJECTORY_ID + DIR_FLAG + traj_name + TXT_ID;
	std::ifstream infile;

	infile.open(traj_path.c_str());

	if (!infile.is_open())
	{
        // LOG_ERROR("Open %s failed!", traj_path.c_str());
		return false;
	}
	//get map name
	getline(infile, str_tempt, '\n');
	ss.clear();
	ss.str(str_tempt);
	ss >> traj.map_name;
	//get trajectory points
	traj.pose.clear();
    traj.pose.reserve(TRAJECTORY_MAX_POINTS);
	Pose3D traj_point;
	while (infile.good() && !infile.eof())
	{
		getline(infile, str_tempt, '\n');
		ss.clear();
		ss.str(str_tempt);
		ss >> traj_point.x >> traj_point.y >> traj_point.theta;
		if (ss.fail())
		{
			continue;
		}
//		LOG_INFO("trajectory[" << count++ << "] = " << str_tempt;
		traj.pose.push_back(traj_point);
	}
    // LOG_INFO("Read %s successfully!", traj_path.c_str());

    ss.clear();
    ss.str("");
	infile.close();
	return true;
}

/*********************************************************************
function: read reflector_map from xml file
	xml file format:
	(1) file name is the same with the root name
	(2) child is <map_name>, <pose0>, <pose1>...
	(3) special part 1: task_id and RFID
		special part 2: pose probability
		special part 3: map origin and global origin
*********************************************************************/
bool CFileInterface::SaveReflectorMapToXml(const std::string &xml_name,
	const ReflectorMapData &reflector_map, const int write_special_part)
{
    std::string file_name = GetDocsPath() + DOCS_MAP_ID + std::string(DIR_FLAG) + xml_name + std::string(XML_ID);
	// LOG_INFO("Saving reflector map to %s..", file_name.c_str());

	m_config_oper->InitConfigFileOperatorForWriting();
	boost::property_tree::ptree *root_tree = m_config_oper->GetRootTree();
	//write general part of reflector_map
	boost::property_tree::ptree root1, root2, root3;
	root1.put<std::string>("map_name", reflector_map.map_name);

	//write special part of reflector_map
	switch (write_special_part)
	{
    case PartPoseAndNode:
		root1.put<int>("task_id", reflector_map.task_id);
		break;
    case PartOrigin:
        root1.put<std::string>("origin_pose_x", NumToString(reflector_map.origin.x, 4));
        root1.put<std::string>("origin_pose_y", NumToString(reflector_map.origin.y, 4));
        root1.put<std::string>("origin_pose_theta", NumToString(reflector_map.origin.theta, 4));
		break;
	case PartUnknown:
	default:
		break;
	}

	size_t size = reflector_map.pose.size();
	for (size_t i = 0; i < size; ++i)
	{
		root2.clear();
        root3.put_value<std::string>(NumToString(reflector_map.pose[i].x, 4));
		root2.add_child("x", root3);
        root3.put_value<std::string>(NumToString(reflector_map.pose[i].y, 4));
		root2.add_child("y", root3);
        root3.put_value<std::string>(NumToString(reflector_map.pose[i].theta, 4));
		root2.add_child("theta", root3);
		root1.add_child("pose" + NumToString(i), root2);
	}
	root_tree->add_child(xml_name, root1);
	m_config_oper->WriteXmlFile(file_name);
	return true;
}

/*********************************************************************
function: read node information from xml file
    xml file format:
    (1) file name is the same with the root name
    (2) child is <map_name>, <pose0>, <pose1>...
    (3) special part:
            PartUnknown: map name
            PartPoseAndNode: map name / task id / pose /node info
            PartOrigin: map name / map origin pose
            PartPoseAndNodeID: map name / pose /node id
            PartNode: map name / node info
*********************************************************************/
int CFileInterface::ReadNodeInfoFromXml(const std::string &dir, const std::string &xml_name,
    ReflectorMapData &reflector_map, const LandmarkSpecialPart read_special_part,
    const std::string &root_name)
{
    std::string xml_root(root_name);
    if(xml_root.empty())
    {
        xml_root = xml_name;
    }
    m_config_oper->SetNodeDirectory(dir);
    if (!m_config_oper->InitConfigFileOperator(xml_name, xml_root))
    {
        return 1;
    }
    if (!m_config_oper->GetParam("map_name", reflector_map.map_name, std::string("map_0")))
    {
        return 2;
    }

    //read special part of reflector_map
    switch (read_special_part)
    {
    case PartPoseAndNode:
        if (!m_config_oper->GetParam("task_id", reflector_map.task_id, 0))
        {
            return 3;
        }
        break;
    case PartOrigin:
        if (!m_config_oper->GetParam("origin_pose_x", reflector_map.origin.x, 0.0))
        {
            return 4;
        }
        if (!m_config_oper->GetParam("origin_pose_y", reflector_map.origin.y, 0.0))
        {
            return 5;
        }
        if (!m_config_oper->GetParam("origin_pose_theta", reflector_map.origin.theta, 0.0))
        {
            return 6;
        }
        break;
    case PartUnknown:
    default:
        break;
    }

    std::deque<std::string> root_queue;
    root_queue.push_back(xml_root);
    if (!m_config_oper->InitConfigFileOperator(xml_name, root_queue))
    {
        return 7;
    }

    size_t child_size = m_config_oper->GetChildNum(root_queue);
//    LOG_INFO("Child num = %d", child_size);
    std::string chile_name("");
    Pose3D pt;
    PathNode node;
    int ret = 0;
    for (size_t i = 0; i < child_size; ++i)
    {
        chile_name = "pose" + NumToString(i);
        root_queue.push_back(chile_name);
        m_config_oper->SetRootQueue(root_queue);

        //read special part of reflector_map
        switch (read_special_part)
        {
        case PartOrigin:
        {
            ret = ReadNodePose(pt);
            if(0 != ret)
            {
                break;
            }
            reflector_map.pose.push_back(pt);
            break;
        }
        case PartPoseAndNode:
        {
            ret = ReadNodePose(pt);
            if(0 != ret)
            {
                break;
            }
            reflector_map.pose.push_back(pt);

            ret = ReadNodeInfo(node);
            if(0 != ret)
            {
                return 8 + RETURN_FALSE_MAX_NUM * ret;
            }
            reflector_map.node.push_back(node);
            break;
        }
        case PartPoseAndNodeID:
        {
            ret = ReadNodePose(pt);
            if(0 != ret)
            {
                break;
            }
            reflector_map.pose.push_back(pt);

            if (!m_config_oper->GetParam("id", node.id, 0))
            {
                return 9;
            }
            reflector_map.node.push_back(node);
            break;
        }
        case PartNode:
        {
            ret = ReadNodeInfo(node);
            if(0 != ret)
            {
                break;
            }
            reflector_map.node.push_back(node);
            break;
        }
        case PartPoseAndNodeIDAndType:
        {
            ret = ReadNodePose(pt);
            if(0 != ret)
            {
                break;
            }
            reflector_map.pose.push_back(pt);

            if (!m_config_oper->GetParam("id", node.id, 0))
            {
                return 9;
            }

            int type = 0;
            if (!m_config_oper->GetParam("motion_type", type, 0))
            {
                return 2;
            }
            if (type >= (int)MotionTypeEnum::MTYPE_NUM)
            {
                return 3;
            }
            node.motion_type = (MotionTypeEnum)type;
            //0903----
            //LOG_INFO("node.motion_type=%d",node.motion_type);
            reflector_map.node.push_back(node);
            break;
        }
        case PartNodeIDAndType:
        {
            if (!m_config_oper->GetParam("id", node.id, 0, false, false))
            {
                break;
            }

            int type = 0;
            if (!m_config_oper->GetParam("motion_type", type, 0))
            {
                return 2;
            }
            if (type >= (int)MotionTypeEnum::MTYPE_NUM)
            {
                return 3;
            }
            node.motion_type = (MotionTypeEnum)type;
            //0903----
            //LOG_INFO("node.motion_type=%d",node.motion_type);
            reflector_map.node.push_back(node);
            break;
        }
        case PartUnknown:
        default:
            break;
        }
        root_queue.pop_back();
    }
//    LOG_INFO("All availabal nodes(size = %d) have been read!", reflector_map.node.size());
    return 0;
}

int CFileInterface::ReadNodePose(Pose3D &pose)
{
    if (!m_config_oper->GetParam("x", pose.x, 0.0, false, false))
    {
        return 1;
    }
    if (!m_config_oper->GetParam("y", pose.y, 0.0))
    {
        return 2;
    }
    if (!m_config_oper->GetParam("theta", pose.theta, 0.0))
    {
        return 3;
    }
    return 0;
}

int CFileInterface::ReadNodeInfo(PathNode &node)
{
    if (!m_config_oper->GetParam("id", node.id, 0, false, false))
    {
        return 1;
    }
//    LOG_INFO("id = %d", node.id);
    int type = 0;
    if (!m_config_oper->GetParam("motion_type", type, 1))
    {
        return 2;
    }
    if (type >= (int)MotionTypeEnum::MTYPE_NUM)
    {
        return 3;
    }
    node.motion_type = (MotionTypeEnum)type;
    if (!m_config_oper->GetParam("static_work1", node.static_work1, 0.))
    {
        return 4;
    }
    if (!m_config_oper->GetParam("static_work2", node.static_work2, 0.))
    {
        return 5;
    }
    if (!m_config_oper->GetParam("target_vel", node.target_vel, 0.))
    {
        return 6;
    }
    if (!m_config_oper->GetParam("vel_angle", node.vel_angle, 0.))
    {
        return 7;
    }
    //是否指定弯道的圆心与半径，一般用于圆弧与直线不相切的情况
    //在指定圆心与转弯半径时，要注意圆心坐标、半径、速度角度均需要人工计算
    //速度角度=360时，车身朝向保持不变，转弯过程中车身朝向与路线夹角会连续变化
    //速度角度在[-180, 180]时，车身朝向与路线夹角恒定
    if (m_config_oper->GetParam("valid_turn_sector", node.valid_turn_sector,
                                false, false, false))
    {
        if (!m_config_oper->GetParam("turn_center_x", node.turn_sector.center.x, 0.))
        {
            return 8;
        }
        if (!m_config_oper->GetParam("turn_center_y", node.turn_sector.center.y, 0.))
        {
            return 9;
        }
        if (!m_config_oper->GetParam("turn_radius", node.turn_sector.radius, 0.))
        {
            return 10;
        }
    }
    else
    {
        node.valid_turn_sector = false;
        node.turn_sector.Reset();
    }
    node.vel_angle = SlamCommon::DegToRad(node.vel_angle);
    node.turn_sector.angle = 0.;

    type = (int)node.navi_type;
    m_config_oper->GetParam("navi_type", type, type, false, false);
    node.navi_type = (SlamCommon::NaviAlgoTypeEnum)type;
    m_config_oper->GetParam("map_name_index", node.map_name_index,
                            node.map_name_index, false, false);
    type = (int)node.localizer_type;
    m_config_oper->GetParam("localizer_type", type, type, false, false);
    node.localizer_type = (SlamCommon::AlgorithmNameEnum)type;
    return 0;
}

/*********************************************************************
function: save current laser pose to txt file
//ph add seedpose 2022/2/12
*********************************************************************/
void CFileInterface::SaveSeedPose(const std::string &pose_name, const Pose3D &agv_pose)
{
    int now = ToUniversal(TimeNow()) % INT_MAX;
    const std::string txt_name = GetDocsPath()
            + DOCS_RECORD_SEEDPOSE_ID + DIR_FLAG + pose_name + TXT_ID;
    std::ofstream pose_file(txt_name, std::ios::out | std::ios::binary);

    std::string line = NumToString(now) + " "
        + NumToString(agv_pose.x) + " "
        + NumToString(agv_pose.y) + " "
        + NumToString(agv_pose.theta) + "\n";
        m_config_oper->WriteStringToFile(pose_file, line);
    pose_file.close();
//     CHECK(pose_file) << "Writing " << txt_name << " failed.");
}

/*********************************************************************
function: save current pose to txt file
*********************************************************************/
void CFileInterface::SaveAGVPose(const std::string &pose_name, const Pose3D &agv_pose)
{
	int now = ToUniversal(TimeNow()) % INT_MAX;
    const std::string txt_name = GetDocsPath()
            + DOCS_RECORD_POSE_ID + DIR_FLAG + pose_name + TXT_ID;
	std::ofstream pose_file(txt_name, std::ios::out | std::ios::binary);

	std::string line = NumToString(now) + " "
		+ NumToString(agv_pose.x) + " "
		+ NumToString(agv_pose.y) + " "
		+ NumToString(agv_pose.theta) + "\n";
		m_config_oper->WriteStringToFile(pose_file, line);
	pose_file.close();
//     CHECK(pose_file) << "Writing " << txt_name << " failed.");
}

/*********************************************************************
function: read record initial pose from txt file
*********************************************************************/
bool CFileInterface::ReadInitialPoseFromTxt(const std::string &dir_name,
    const std::string &txt_name, int &time, Pose3D &init_pose)
{
	std::string str_tempt;
	std::stringstream str_stream;
    std::string dir(dir_name);
    AddDirectoryFlag(dir);
    std::string pose_path = m_cur_path + dir + txt_name + TXT_ID;
	std::ifstream infile;

	infile.open(pose_path.c_str());

	if (!infile.is_open())
	{
        LOG(WARNING) << "Open %s failed!" << pose_path.c_str();
		return false;
	}

	while (infile.good() && !infile.eof())
	{
		getline(infile, str_tempt, '\n');
		if (str_tempt.empty())
		{
			continue;
		}
		str_stream.clear();
		str_stream.str(str_tempt);
		
		str_stream >> time >> init_pose.x >> init_pose.y >> init_pose.theta;

		if (str_stream.fail())
		{
             LOG(WARNING) << "String stream from reading %s failed! Continue." << pose_path.c_str();
			continue;
		}
	}

	infile.close();
    str_stream.clear();
    str_stream.str("");
	// LOG_INFO("Read %s successfully!", pose_path.c_str());
	return true;
}

/*********************************************************************
function: read initial pose from txt file
*********************************************************************/
bool CFileInterface::ReadInitialPoseFromTxt(const std::string &dir_name,
    const std::string &txt_name, Pose3D &init_pose)
{
    std::string str_tempt;
    std::stringstream str_stream;
    std::string dir(dir_name);
    AddDirectoryFlag(dir);
    std::string pose_path = m_cur_path + dir + txt_name + TXT_ID;
    std::ifstream infile;

    infile.open(pose_path.c_str());

    if (!infile.is_open())
    {
        LOG(WARNING) << "Open %s failed!" << pose_path.c_str();
        return false;
    }

    while (infile.good() && !infile.eof())
    {
        getline(infile, str_tempt, '\n');
        if (str_tempt.empty())
        {
            continue;
        }
        str_stream.clear();
        str_stream.str(str_tempt);

        str_stream >> init_pose.x >> init_pose.y >> init_pose.theta;

        if (str_stream.fail())
        {
            LOG(WARNING) << "String stream from reading %s failed! Continue." << pose_path.c_str();
            continue;
        }
    }

    infile.close();
    str_stream.clear();
    str_stream.str("");
    // LOG_INFO("Read %s successfully!", pose_path.c_str());
    return true;
}


///////////xcc add 0331 for code reader
/*********************************************************************
function: read record initial pose from txt file
*********************************************************************/
bool CFileInterface::ReadCodePoseMapFromTxt(const std::string &dir_name,
const std::string &txt_name, CodePoseMap &code_pose_map)
{
    std::string str_tempt;
    std::stringstream str_stream;
    std::string dir(dir_name);
    AddDirectoryFlag(dir);
    std::string pose_path = m_cur_path + dir + txt_name + TXT_ID;
    std::ifstream infile;

    infile.open(pose_path.c_str());

    if (!infile.is_open())
    {
        LOG(WARNING) << "Open %s failed!" << pose_path.c_str();
        return false;
    }
    //code_pose_map.file_Id = int(txt_name.back());
    //LOG_DEBUG("input fileID = %d",code_pose_map.file_Id);
    //int i =0;
    long id_value=0;
    Pose3D pose_Value;
    while (infile.good() && !infile.eof())
    {
        getline(infile, str_tempt, '\n');
        if (str_tempt.empty())
        {
            continue;
        }
        str_stream.clear();
        str_stream.str(str_tempt);

        str_stream >> id_value >> pose_Value.x >> pose_Value.y >> pose_Value.theta;

        if (str_stream.fail())
        {
            LOG(WARNING) << "String stream from reading %s failed! Continue." << pose_path.c_str();
            continue;
        }
        else
        {
            int ret = code_pose_map.push_codePosePar(id_value,pose_Value);
            LOG_INFO("push one pair value sucess, the list size is %d",ret);
        }
    }

    infile.close();
    str_stream.clear();
    str_stream.str("");
    LOG_INFO("Read %s successfully!", pose_path.c_str());
    return true;
}
///
/*********************************************************************
function: save poses, which is Pose3D format to txt file
*********************************************************************/
void CFileInterface::WriteInitPoseToTxt(const std::string &dir_name,
    const std::string &pose_name, const Pose3D &pose)
{
    std::string dir(dir_name);
    AddDirectoryFlag(dir);
    const std::string txt_name = m_cur_path + dir + pose_name + TXT_ID;
    std::ofstream pose_file(txt_name, std::ios::out | std::ios::binary);

    std::string line("");
    line = NumToString(pose.x) + " " + NumToString(pose.y) + " " + NumToString(pose.theta);
    m_config_oper->WriteStringToFile(pose_file, line);
    pose_file.close();
    LOG_INFO("Write init pose %s successfully!", txt_name.c_str());
}

/*********************************************************************
function: save poses, which is Pose3D format to txt file
*********************************************************************/
void CFileInterface::SavePosesToFile(const std::string &pose_name,
    const std::vector<Pose3D> &poses)
{
    const std::string txt_name = GetDocsPath() + pose_name + TXT_ID;
    std::ofstream pose_file(txt_name, std::ios::out | std::ios::binary | std::ios::app);

    std::string line("");
    size_t size = poses.size();
    for (size_t i = 0; i < size; ++i)
    {
        line = NumToString(poses[i].x) + " " + NumToString(poses[i].y) + " " + NumToString(poses[i].theta) + "\n";
        m_config_oper->WriteStringToFile(pose_file, line);
    }
    pose_file.close();
//     CHECK(pose_file) << "Writing " << txt_name << " failed.");
   LOG_INFO("Save %s successfully!", txt_name.c_str());
}

/*********************************************************************
function: save each point cloud to a txt file
*********************************************************************/
void CFileInterface::SaveMultiPosesToFile(const std::string &pose_name,
    const std::vector<std::vector<Pose3D>> &multi_poses_container)
{
    size_t container_size = multi_poses_container.size();
    std::string file_name("");
    for (size_t i = 0; i < container_size; ++i)
    {
        file_name.clear();
        file_name = pose_name + std::string(UNDERLINE_FLAG)
                + NumToString(i);
        file_name = GetDocsPath() + pose_name
                + std::string(DIR_FLAG) + file_name + std::string(TXT_ID);
		std::ofstream poses_file(file_name, std::ios::out | std::ios::binary);

        std::vector<Pose3D> multi_poses = multi_poses_container[i];
		std::string line("");
		for (std::vector<Pose3D>::iterator pose = multi_poses.begin();
			pose != multi_poses.end(); ++pose)
		{
			line += NumToString(pose->x) + " "
				+ NumToString(pose->y) + " "
				+ NumToString(pose->theta) + "\n";
		}
		m_config_oper->WriteStringToFile(poses_file, line);
		poses_file.close();
//         CHECK(poses_file) << "Writing " << file_name << " failed!");
	}
    LOG_INFO("Saved %s successfully!", file_name.c_str());
}

/*********************************************************************
function: load laser hit data from file
*********************************************************************/
bool CFileInterface::ReadPose3DFromFile(const std::string &file_name,
	std::vector<Pose3D> &poses_container)
{
	const std::string file_path = GetDocsPath() + file_name + std::string(TXT_ID);

	std::string str_tempt;
	std::stringstream ss;
	std::ifstream infile;
	Pose3D pose;

	infile.open(file_path.c_str());

	if (!infile.is_open())
	{
		// LOG_ERROR("Open %s failed!", file_path.c_str());
		return false;
	}

	poses_container.clear();
	poses_container.reserve(3000);
	while (infile.good() && !infile.eof())
	{
		getline(infile, str_tempt, '\n');
		if (str_tempt.empty())
		{
			continue;
		}
		ss.clear();
		ss.str(str_tempt);
		ss >> pose.x >> pose.y >> pose.theta;
		poses_container.push_back(pose);
	}

	infile.close();
    ss.clear();
    ss.str("");
	LOG_INFO("Read %s successfully!", file_path.c_str());
	return true;
}

#ifdef WIN32
void CFileInterface::TraverseFilesInsideDirectory(const std::string &path, std::vector<std::string> &files)
{
    files.clear();
    long file_t = 0;
    struct _finddata_t file_info;
    std::string path_name;

    if ((file_t = _findfirst(path_name.assign(path).append("\\*").c_str(), &file_info)) == -1)
    {
        return;
    }
    do
    {
        if (file_info.attrib & _A_SUBDIR)
        {
            std::string name_t = std::string(file_info.name);
            if (name_t != DOUBLE_DOT_FLAG && name_t != SINGLE_DOT_FLAG)
            {
                TraverseFilesInOneDirectory(path + "\\" + name_t, files);
            }
        }
        else
        {
            files.push_back(file_info.name);
        }
    } while (_findnext(file_t, &file_info) == 0);
    _findclose(file_t);
}

void CFileInterface::TraverseFilesInOneDirectory(const std::string &path, std::vector<std::string> &files)
{
    files.clear();
    long file_t = 0;
    struct _finddata_t file_info;
    std::string path_name;

    if ((file_t = _findfirst(path_name.assign(path).append("\\*").c_str(), &file_info)) == -1)
    {
        return;
    }
    do
    {
        if (!(file_info.attrib & _A_SUBDIR))
        {
            files.push_back(file_info.name);
        }
    } while (_findnext(file_t, &file_info) == 0);
    _findclose(file_t);
}

void CFileInterface::DeleteAllFilesInDirectory(const std::string &path)
{
    long file_t = 0;
    struct _finddata_t file_info;
    std::string path_name;

    if ((file_t = _findfirst(path_name.assign(path).append("\\*").c_str(), &file_info)) == -1)
    {
        return;
    }
    do
    {
        if (file_info.attrib & _A_SUBDIR)
        {
            std::string name_t = std::string(file_info.name);
            if (name_t != DOUBLE_DOT_FLAG && name_t != SINGLE_DOT_FLAG)
            {
                DeleteAllFilesInDirectory(path + "\\" + name_t);
            }
        }
        else
        {
            std::string cur_path(path);
            AddDirectoryFlag(cur_path);
            LOG_INFO("cur path: %s, file name: %s", cur_path.c_str(), file_info.name);
            m_config_oper->DeleteFile(cur_path + file_info.name);
        }
    } while (_findnext(file_t, &file_info) == 0);
    _findclose(file_t);
}

#else
void CFileInterface::TraverDir(const std::string &path, std::vector<std::string> &files)
{
    int depth = 1;
    const char * m_path = path.c_str();
    files = TraveDir(m_path, depth);
}

std::vector<std::string> CFileInterface::TraveDir(const char *path, int depth)
{
    DIR *d;
    struct dirent *file;
    struct stat sb;
    std::vector<std::string> file_names;

    if(!(d = opendir(path)))
    {
        printf("error opendir %s!!!/n",path);
        return file_names;
    }
    while((file = readdir(d)) != NULL)
    {
        if(strncmp(file->d_name, SINGLE_DOT_FLAG, 1) == 0)
            continue;
        //strcpy(filename[len++], file->d_name);
        file_names.push_back(file->d_name);
        if(stat(file->d_name, &sb) >= 0 && S_ISDIR(sb.st_mode) && depth <= 3)
        {
            TraveDir(file->d_name, depth + 1);
        }
    }
    closedir(d);
    return file_names;
}

int CFileInterface::TraverseDirectory(const char* path)
{
	DIR *d;
	struct dirent *file;
	struct stat buf;

	if (!(d = opendir(path)))
	{
        // LOG_ERROR("Cannot open directory: %s", path);
		return -1;
	}
	//Add this, so that it can scan the children dir
	if (-1 == chdir(path))
    {
        // LOG_ERROR("Change directory error: not a dir or access!");
		return -1;
	}

	while ((file = readdir(d)) != NULL)
	{
        if (strncmp(file->d_name, SINGLE_DOT_FLAG, 1) == 0)
			continue;
		if (stat(file->d_name, &buf) >= 0 && !S_ISDIR(buf.st_mode))
		{
            // LOG_INFO("File name = %s", file->d_name);
            // LOG_INFO("File size = %d", buf.st_size);
            // LOG_INFO("File last modify time = %d", buf.st_mtime);
		}
	}
	closedir(d);
	return 0;
}

void CFileInterface::DeleteAllFilesInDirectory(const std::string &dir)
{
    std::string cur_dir(dir);
    DIR *dp = nullptr;
	struct dirent *entry = nullptr;
	struct stat statbuf;

    if ((dp = opendir(cur_dir.c_str())) == NULL)
	{
        LOG_ERROR("Cannot open directory: %s", cur_dir.c_str());
		return;
	}

	//change direcotry
    if (-1 == chdir(cur_dir.c_str()))
	{
        LOG_ERROR("Change directory error: not a dir or access!");
		return;
	}

	//traverse this directory, and delete all files in it
	while ((entry = readdir(dp)) != NULL)
	{
		//read the file type in this directory
		lstat(entry->d_name, &statbuf);

		//ignore directory flat
        if (strcmp(entry->d_name, SINGLE_DOT_FLAG) == 0 || strcmp(entry->d_name, DOUBLE_DOT_FLAG) == 0)
		{
            LOG_INFO("Read directory: %s, continue!", entry->d_name);
			continue;
		}

		//if it is a directory, go into it
		if (S_ISDIR(statbuf.st_mode))
		{
            LOG_INFO("Read directory %s, enter it!", entry->d_name);
			//change directory
			if (-1 == chdir(entry->d_name))
			{
				LOG_ERROR("Change directory failed!");
				return;
			}
			//traverse into the deeper directory
            DeleteAllFilesInDirectory(SINGLE_DOT_FLAG);
			//return to the last directory
            if (-1 == chdir(DOUBLE_DOT_FLAG))
			{
				LOG_ERROR("Return to the last directory failed!");
				return;
			}
		}

		//if it is a file, delete it
		if (S_ISREG(statbuf.st_mode))
		{
            LOG_INFO("Remove file %s!", entry->d_name);
			remove(entry->d_name);
		}
	}

    //change into the origin direcotry
    if (-1 == chdir(m_cur_path.c_str()))
    {
        LOG_ERROR("Change directory(%s) error: not a dir or access!", m_cur_path.c_str());
        return;
    }
	closedir(dp);
    dp = nullptr;
    entry = nullptr;
}

int CFileInterface::ExtractAllfileNameInDir(const std::string &dir,
                                            std::vector<std::string> &file_names)
{
    DIR *dp;
    struct dirent *entry = nullptr;
    struct stat statbuf;

    if ((dp = opendir(dir.c_str())) == NULL)
    {
        LOG_ERROR("Cannot open directory: %s", dir.c_str());
        return 1;
    }

    //change direcotry
    if (-1 == chdir(dir.c_str()))
    {
        LOG_ERROR("Change directory error: not a dir or access!");
        return 2;
    }

    //traverse this directory, and delete all files in it
    while ((entry = readdir(dp)) != NULL)
    {
        //read the file type in this directory
        lstat(entry->d_name, &statbuf);

        //ignore directory flat
        if (strcmp(entry->d_name, SINGLE_DOT_FLAG) == 0 || strcmp(entry->d_name, DOUBLE_DOT_FLAG) == 0)
        {
            LOG_INFO("read directory: %s, continue!", entry->d_name);
            continue;
        }

        //if it is a file, delete it
        if (S_ISREG(statbuf.st_mode))
        {
            LOG_INFO("Find file %s", entry->d_name);
            file_names.push_back(entry->d_name);
        }
    }
    closedir(dp);
    return 0;
}

int CFileInterface::ExtractAllDirNameInDir(const std::string &dir,
                                            std::vector<std::string> &dir_names)
{
    DIR *dp;
    struct dirent *entry = nullptr;
    struct stat statbuf;

    if ((dp = opendir(dir.c_str())) == NULL)
    {
        // LOG_ERROR("Cannot open directory: %s", dir.c_str());
        return 1;
    }

    //change direcotry
    if (-1 == chdir(dir.c_str()))
    {
        // LOG_ERROR("Change directory error: not a dir or access!");
        return 2;
    }

    //traverse this directory, and delete all files in it
    while ((entry = readdir(dp)) != NULL)
    {
        //read the file type in this directory
        lstat(entry->d_name, &statbuf);

        //ignore directory flat
        if (strcmp(entry->d_name, SINGLE_DOT_FLAG) == 0
                || strcmp(entry->d_name, DOUBLE_DOT_FLAG) == 0)
        {
//            LOG_INFO("read directory: %s, continue!", entry->d_name);
            continue;
        }

        //if it is a dir, store it
        if (S_ISDIR(statbuf.st_mode))
        {
//            LOG_INFO("Find dir %s", entry->d_name);
            dir_names.push_back(entry->d_name);
        }
    }
    closedir(dp);
    return 0;
}

#endif

/*********************************************************************
function: save laser range to file
*********************************************************************/
void CFileInterface::SaveLaserRangeDataToFile(const std::string &laser_hits_name,
	const std::vector<SlamLaserScanData> &laser_hits)
{
	const std::string file_name = GetDocsPath() + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
		+ std::string(DIR_FLAG) + std::string(DOCS_LASER_HIT_ID)
		+ std::string(DIR_FLAG) + laser_hits_name + std::string(TXT_ID);
	std::ofstream laser_hits_file(file_name, std::ios::out | std::ios::binary);

	std::string line("");
	size_t container_size = laser_hits.size(), range_size = 0;
	for (size_t i = 0; i < container_size; ++i)
	{
		range_size = laser_hits[i].ranges.size();
		line = NumToString(ToUniversal(laser_hits[i].header.time)) + " ";
		for (size_t j = 0; j < range_size; j++)
		{
			line += NumToString(laser_hits[i].ranges[j]) + " ";
		}
		line += "\n";
		m_config_oper->WriteStringToFile(laser_hits_file, line);
	}

	laser_hits_file.close();
    ////LOG_CHECK(laser_hits_file);
	LOG_INFO("Save %s successfully!", file_name.c_str());
}

/*********************************************************************
function: load laser range data from file,
    param: laser_hits_name = dir + file name
*********************************************************************/
bool CFileInterface::ReadLaserRangeDataFromFile(const std::string &laser_hits_name,
    std::queue<SlamLaserScanData> &laser_queue)
{
    const std::string file_name = GetDocsPath() + laser_hits_name;

    std::string str_tempt;
    std::stringstream ss;
    std::ifstream infile;
    SlamLaserScanData laser_data;
    double range = 0.0;
    uint64_t time = 0;

    infile.open(file_name.c_str());

    if (!infile.is_open())
    {
        // LOG_ERROR("Open %s failed!", file_name.c_str());
        return false;
    }

    while (infile.good() && !infile.eof())
    {
        getline(infile, str_tempt, '\n');
        if (str_tempt.empty())
        {
            continue;
        }
        ss.clear();
        ss.str(str_tempt);

        ss >> time;
        laser_data.header.time = FromUniversal(time);

        laser_data.ranges.clear();
        laser_data.ranges.reserve(4000);
        while (!ss.eof())
        {
            ss >> range;
            if (ss.fail())
            {
                break;
            }
            laser_data.ranges.push_back(range);
        }
        laser_queue.push(laser_data);
    }

    infile.close();
    ss.clear();
    ss.str("");
	LOG_INFO("Read %s successfully!", file_name.c_str());
    return true;
}

/*********************************************************************
function: save laser range and intensity to file
*********************************************************************/
void CFileInterface::SaveLaserRangeAndItensityDataToFile(
        const std::string &laser_hits_name,
        const std::vector<SlamLaserScanData> &laser_hits)
{
    const std::string file_name = GetDocsPath()
            + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
            + std::string(DIR_FLAG) + std::string(DOCS_LASER_HIT_ID)
            + std::string(DIR_FLAG) + laser_hits_name + std::string(TXT_ID);
    std::ofstream laser_hits_file(file_name, std::ios::out | std::ios::binary);

    std::string line("");
    size_t container_size = laser_hits.size(), range_size = 0;
    for (size_t i = 0; i < container_size; ++i)
    {
        range_size = laser_hits[i].ranges.size();
        line = NumToString(ToUniversal(laser_hits[i].header.time)) + " ";
        for (size_t j = 0; j < range_size; j++)
        {
            line = line + NumToString(laser_hits[i].ranges[j]) + " "
                        + NumToString(laser_hits[i].intensities[j]) + " ";
        }
        line += "\n";
        m_config_oper->WriteStringToFile(laser_hits_file, line);
    }

    laser_hits_file.close();
    ////LOG_CHECK(laser_hits_file);
	LOG_INFO("Save %s successfully!", file_name.c_str());
}

/*********************************************************************
function: load laser range and intensity data from file,
	param: laser_hits_name = dir + file name
*********************************************************************/
bool CFileInterface::ReadLaserRangeAndItensityDataFromFile(
        const std::string &laser_hits_name,
        std::deque<SlamLaserScanData> &laser_queue)
{
	const std::string file_name = GetDocsPath() + laser_hits_name;

	std::string str_tempt;
	std::stringstream ss;
	std::ifstream infile;
	SlamLaserScanData laser_data;
	double range = 0.0;
    int intensity = 0;
	uint64_t time = 0;

	infile.open(file_name.c_str());

	if (!infile.is_open())
	{
		// LOG_ERROR("Open %s failed!", file_name.c_str());
		return false;
	}

	while (infile.good() && !infile.eof())
	{
		getline(infile, str_tempt, '\n');
		if (str_tempt.empty())
		{
			continue;
		}
		ss.clear();
		ss.str(str_tempt);

		ss >> time;
		laser_data.header.time = FromUniversal(time);

		laser_data.ranges.clear();
        laser_data.ranges.reserve(4000);
        laser_data.intensities.clear();
        laser_data.intensities.reserve(4000);
		while (!ss.eof())
		{
			ss >> range;
            ss >> intensity;
            if (ss.fail())
            {
                break;
            }
			laser_data.ranges.push_back(range);
            laser_data.intensities.push_back(intensity);
		}
        laser_queue.push_back(laser_data);
	}

	infile.close();
    ss.clear();
    ss.str("");
	LOG_INFO("Read %s successfully!", file_name.c_str());
	return true;
}

/*********************************************************************
function: save laser range key configuration paramters to xml file
*********************************************************************/
void CFileInterface::SaveLaserKeyParams(const SlamLaserScanData &laser_data)
{
	const std::string xml_name = GetDocsPath() + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
		+ std::string(DIR_FLAG) + std::string(DOCS_LASER_HIT_ID) + std::string(DIR_FLAG)
		+ std::string(DOCS_LASER_HIT_ID) + std::string(XML_ID);

	m_config_oper->InitConfigFileOperatorForWriting();

	m_config_oper->SetParam("frame_id", laser_data.header.frame_id);
    m_config_oper->SetParam("frequency", laser_data.header.frequency);
	m_config_oper->SetParam("sensor_id", laser_data.sensor_id);
	m_config_oper->SetParam("angle_min", laser_data.angle_min);
	m_config_oper->SetParam("angle_max", laser_data.angle_max);
	m_config_oper->SetParam("angle_increment", laser_data.angle_increment);
	m_config_oper->SetParam("range_min", laser_data.range_min);
	m_config_oper->SetParam("range_max", laser_data.range_max);
    m_config_oper->SetParam("time_increment", laser_data.time_increment);

    m_config_oper->WriteXmlFile(xml_name);//write to xml file
	m_config_oper->UninitConfigFileOperator();

	LOG_INFO("Saving %s successfully!", xml_name.c_str());
}

/*********************************************************************
function: read laser range key configuration paramters from xml file
*********************************************************************/
void CFileInterface::ReadLaserKeyParams(SlamLaserScanData *laser_data, std::string &map_name)
{
	std::string file_path(GetDocsPath() + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
		+ std::string(DIR_FLAG) + std::string(DOCS_LASER_HIT_ID));
	m_config_oper->SetNodeDirectory(file_path);
	if (!m_config_oper->InitConfigFileOperator(DOCS_LASER_HIT_ID))
	{
		return;
	}

    m_config_oper->GetParam("frame_id", laser_data->header.frame_id,
                            std::string(LASER_SCAN_ID + std::string(SENSOR_FRAME_ID)));
    m_config_oper->GetParam("frequency", laser_data->header.frequency, 0.);
	m_config_oper->GetParam("sensor_id", laser_data->sensor_id, std::string(LASER_SCAN_ID));
	m_config_oper->GetParam("angle_min", laser_data->angle_min, -95.0);
	m_config_oper->GetParam("angle_max", laser_data->angle_max, 95.0);
	m_config_oper->GetParam("angle_increment", laser_data->angle_increment, 0.05);
	m_config_oper->GetParam("range_min", laser_data->range_min, 0.5);
	m_config_oper->GetParam("range_max", laser_data->range_max, 80.0);
    m_config_oper->GetParam("time_increment", laser_data->time_increment, 0.0);
    m_config_oper->GetParam("map_name", map_name, std::string("1"));
	m_config_oper->UninitConfigFileOperator();
	LOG_INFO("Read %s successfully!", file_path.c_str());
}

/*********************************************************************
function: save IMU data to file
*********************************************************************/
void CFileInterface::SaveIMUDataToFile(const std::string &imu_name,
	const std::vector<SlamIMUData> &imu_data)
{
	const std::string file_name = GetDocsPath() + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
		+ std::string(DIR_FLAG) + std::string(IMU_ID)
		+ std::string(DIR_FLAG) + imu_name + std::string(TXT_ID);
	std::ofstream imu_file(file_name, std::ios::out | std::ios::binary);

	std::string line("");
	size_t container_size = imu_data.size();
	for (size_t i = 0; i < container_size; ++i)
	{
		line = NumToString(ToUniversal(imu_data[i].header.time))
			+ " " + NumToString(imu_data[i].angular_velocity.x())
			+ " " + NumToString(imu_data[i].angular_velocity.y())
			+ " " + NumToString(imu_data[i].angular_velocity.z())
			+ " " + NumToString(imu_data[i].linear_acceleration.x())
			+ " " + NumToString(imu_data[i].linear_acceleration.y())
			+ " " + NumToString(imu_data[i].linear_acceleration.z())
			+ " " + NumToString(imu_data[i].orientation.x())
			+ " " + NumToString(imu_data[i].orientation.y())
			+ " " + NumToString(imu_data[i].orientation.z())
			+ " " + NumToString(imu_data[i].orientation.w()) + "\n";
		m_config_oper->WriteStringToFile(imu_file, line);
	}

	imu_file.close();
    ////LOG_CHECK(imu_file);
}

/*********************************************************************
function: load IMU data from file
*********************************************************************/
bool CFileInterface::ReadIMUDataFromFile(const std::string &imu_name,
	std::queue<SlamIMUData> &imu_queue)
{
	const std::string file_name = GetDocsPath() + imu_name;

	std::string str_tempt;
	std::stringstream ss;
	std::ifstream infile;
	SlamIMUData imu_data;
	double data_tempt = 0.0;
	uint64_t time = 0;

	infile.open(file_name.c_str());

	if (!infile.is_open())
	{
		// LOG_ERROR("Open %s failed!", file_name.c_str());
		return false;
	}

	while (infile.good() && !infile.eof())
	{
		getline(infile, str_tempt, '\n');
		if (str_tempt.empty())
		{
			continue;
		}
		ss.clear();
		ss.str(str_tempt);

		ss >> time;
		imu_data.header.time = FromUniversal(time);
		ss >> data_tempt;
		imu_data.angular_velocity.x() = data_tempt;
		ss >> data_tempt;
		imu_data.angular_velocity.y() = data_tempt;
		ss >> data_tempt;
		imu_data.angular_velocity.z() = data_tempt;
		ss >> data_tempt;
		imu_data.linear_acceleration.x() = data_tempt;
		ss >> data_tempt;
		imu_data.linear_acceleration.y() = data_tempt;
		ss >> data_tempt;
		imu_data.linear_acceleration.z() = data_tempt;
		ss >> data_tempt;
		imu_data.orientation.x() = data_tempt;
		ss >> data_tempt;
		imu_data.orientation.y() = data_tempt;
		ss >> data_tempt;
		imu_data.orientation.z() = data_tempt;
		ss >> data_tempt;
		imu_data.orientation.w() = data_tempt;

        imu_data.euler_angle = QuaterniondToEuler(imu_data.orientation);

		imu_queue.push(imu_data);

		if (ss.fail())
		{
			// LOG_WARNING("String stream from reading %s failed! Continue.", file_name.c_str());
			continue;
		}
	}

	infile.close();
    ss.clear();
    ss.str("");
	LOG_INFO("Read %s successfully!", file_name.c_str());
	return true;
}

/*********************************************************************
function: save IMU key configuration paramters to xml file
*********************************************************************/
void CFileInterface::SaveIMUKeyParams(const SlamIMUData &imu_data)
{
	const std::string xml_name = GetDocsPath() + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
		+ std::string(DIR_FLAG) + std::string(IMU_ID) + std::string(DIR_FLAG)
		+ std::string(IMU_ID) + std::string(XML_ID);

	m_config_oper->InitConfigFileOperatorForWriting();

	m_config_oper->SetParam("frame_id", imu_data.header.frame_id);
    m_config_oper->SetParam("frequency", imu_data.header.frequency);
	m_config_oper->SetParam("sensor_id", imu_data.sensor_id);

	m_config_oper->WriteXmlFile(xml_name);//write into xml file
	m_config_oper->UninitConfigFileOperator();

	LOG_INFO("Saving %s successfully!", xml_name.c_str());
}

/*********************************************************************
function: read IMU key configuration paramters from xml file
*********************************************************************/
void CFileInterface::ReadIMUKeyParams(SlamIMUData *imu_data)
{
	std::string file_path(GetDocsPath() + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
		+ std::string(DIR_FLAG) + std::string(IMU_ID));
	m_config_oper->SetNodeDirectory(file_path);
	if (!m_config_oper->InitConfigFileOperator(IMU_ID))
	{
		return;
	}

    m_config_oper->GetParam("frame_id", imu_data->header.frame_id,
                            std::string(IMU_ID + std::string(SENSOR_FRAME_ID)));
    m_config_oper->GetParam("frequency", imu_data->header.frequency, 0.);
	m_config_oper->GetParam("sensor_id", imu_data->sensor_id, std::string(IMU_ID));

	m_config_oper->UninitConfigFileOperator();
	LOG_INFO("Read %s successfully!", file_path.c_str());
}

/*********************************************************************
function: save odometry data to file
*********************************************************************/
void CFileInterface::SaveOdometryDataToFile(const std::string &odometry_name,
    const std::vector<SlamOdometryPoseData> &data)
{
    const std::string file_name = GetDocsPath() + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
        + std::string(DIR_FLAG) + std::string(ODOMETRY_ID)
        + std::string(DIR_FLAG) + odometry_name + std::string(TXT_ID);
    std::ofstream odometry_file(file_name, std::ios::out | std::ios::binary);

    std::string line("");
    size_t container_size = data.size();
    for (size_t i = 0; i < container_size; ++i)
    {
        line = NumToString(ToUniversal(data[i].header.time))
            + " " + NumToString(data[i].pose.translation.x())
            + " " + NumToString(data[i].pose.translation.y())
            + " " + NumToString(data[i].pose.translation.z())
            + " " + NumToString(data[i].pose.rotation.x())
            + " " + NumToString(data[i].pose.rotation.y())
            + " " + NumToString(data[i].pose.rotation.z()) + "\n";
        m_config_oper->WriteStringToFile(odometry_file, line);
    }

    odometry_file.close();
    //LOG_CHECK(odometry_file);
}

/*********************************************************************
function: load odometry data from file
*********************************************************************/
bool CFileInterface::ReadOdometryDataFromFile(const std::string &odometry_name,
    std::queue<SlamOdometryPoseData> &data_queue)
{
    const std::string file_name = GetDocsPath() + odometry_name;

    std::string str_tempt;
    std::stringstream ss;
    std::ifstream infile;
    SlamOdometryPoseData odometry_data;
    double data_tempt = 0.0;
    uint64_t time = 0;

    infile.open(file_name.c_str());

    if (!infile.is_open())
    {
        LOG_ERROR("Open %s failed!", file_name.c_str());
        return false;
    }

    while (infile.good() && !infile.eof())
    {
        getline(infile, str_tempt, '\n');
        if (str_tempt.empty())
        {
            continue;
        }
        ss.clear();
        ss.str(str_tempt);

        ss >> time;
        odometry_data.header.time = FromUniversal(time);
        ss >> data_tempt;
        odometry_data.pose.translation.x() = data_tempt;
        ss >> data_tempt;
        odometry_data.pose.translation.y() = data_tempt;
        ss >> data_tempt;
        odometry_data.pose.translation.z() = data_tempt;
        ss >> data_tempt;
        odometry_data.pose.rotation.x() = data_tempt;
        ss >> data_tempt;
        odometry_data.pose.rotation.y() = data_tempt;
        ss >> data_tempt;
        odometry_data.pose.rotation.z() = data_tempt;

        data_queue.push(odometry_data);

        if (ss.fail())
        {
            LOG(WARNING) << "String stream from reading %s failed! Continue." << file_name.c_str();
            continue;
        }
    }

    infile.close();
    ss.clear();
    ss.str("");
	LOG_INFO("Read %s successfully!", file_name.c_str());
    return true;
}

/*********************************************************************
function: save odometry key configuration paramters to xml file
*********************************************************************/
void CFileInterface::SaveOdometryKeyParams(const SlamOdometryPoseData &data)
{
    const std::string xml_name = GetDocsPath() + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
        + std::string(DIR_FLAG) + std::string(ODOMETRY_ID) + std::string(DIR_FLAG)
        + std::string(ODOMETRY_ID) + std::string(XML_ID);

    m_config_oper->InitConfigFileOperatorForWriting();

    m_config_oper->SetParam("frame_id", data.header.frame_id);
    m_config_oper->SetParam("frequency", data.header.frequency);
    m_config_oper->SetParam("sensor_id", data.sensor_id);

    m_config_oper->WriteXmlFile(xml_name);//write into xml file
    m_config_oper->UninitConfigFileOperator();

	LOG_INFO("Saving %s successfully!", xml_name.c_str());
}

/*********************************************************************
function: read odometry key configuration paramters from xml file
*********************************************************************/
void CFileInterface::ReadOdometryKeyParams(SlamOdometryPoseData *data)
{
    std::string file_path(GetDocsPath() + std::string(DOCS_OFFLINE_MAPPING_DATA_ID)
        + std::string(DIR_FLAG) + std::string(ODOMETRY_ID));
    m_config_oper->SetNodeDirectory(file_path);
    if (!m_config_oper->InitConfigFileOperator(ODOMETRY_ID))
    {
        return;
    }

    m_config_oper->GetParam("frame_id", data->header.frame_id,
                            std::string(ODOMETRY_ID + std::string(SENSOR_FRAME_ID)));
    m_config_oper->GetParam("frequency", data->header.frequency, 0.);
    m_config_oper->GetParam("sensor_id", data->sensor_id, std::string(ODOMETRY_ID));

    m_config_oper->UninitConfigFileOperator();
	LOG_INFO("Read %s successfully!", file_path.c_str());
}

bool CFileInterface::ReadVersionInfo(SlamCommon::ProgramVersionEnum &pv, std::string &fv)
{
    m_config_oper->SetNodeDirectory(DIR_CONFIGURATION_FILE_ID + std::string(DIR_FLAG) + CFG_SLAM_COMMON_ID);
    if(!m_config_oper->InitConfigFileOperator(XML_VERSION_INFO_ID, XML_VERSION_INFO_ID))
    {
        return false;
    }
    int pv_t = 0;
    m_config_oper->GetParam("program_version", pv_t, 0);
    pv = (SlamCommon::ProgramVersionEnum)pv_t;
    m_config_oper->GetParam("firmware_version", fv, std::string("1.0"));
    m_config_oper->UninitConfigFileOperator();
    return true;
}

bool CFileInterface::WriteVersionInfo(const SlamCommon::ProgramVersionEnum &pv, const std::string &fv)
{
    SlamCommon::ProgramVersionEnum pv_origin(SlamCommon::PROGRAMVERSION_NONE);
    std::string fv_origin("");
    if(!ReadVersionInfo(pv_origin, fv_origin))
    {
        return false;
    }

    m_config_oper->InitConfigFileOperatorForWriting();
    boost::property_tree::ptree *root_tree = m_config_oper->GetRootTree();
    boost::property_tree::ptree root1;
    if(pv != SlamCommon::PROGRAMVERSION_NONE)
    {
        root1.put<int>("program_version", (int)pv);
    }
    else
    {
        root1.put<int>("program_version", (int)pv_origin);
    }
    if(fv != "")
    {
        root1.put<std::string>("firmware_version", fv);
    }
    else
    {
        root1.put<std::string>("firmware_version", fv_origin);
    }
    const std::string xml_name = m_cur_path + DIR_CONFIGURATION_FILE_ID + std::string(DIR_FLAG) + CFG_SLAM_COMMON_ID
            + std::string(DIR_FLAG) + std::string(XML_VERSION_INFO_ID) + std::string(XML_ID);
    root_tree->add_child(XML_VERSION_INFO_ID, root1);
    m_config_oper->WriteXmlFile(xml_name);//write to xml
    m_config_oper->UninitConfigFileOperator();
    return true;
}

bool CFileInterface::ReadFile(const std::string &file_name, std::ifstream &file_stream, uint64_t &file_size)
{
    file_size = LOG_FILE_SIZE(file_name);
    LOG_INFO("read %s, size = %ld", file_name.c_str(), file_size);
    file_stream.open(file_name.c_str(), std::ios::binary);
    if(!file_stream.is_open())
    {
        return false;
    }
    return true;
}

int CFileInterface::ReadMappingTableFromXml(const std::string &dir, const std::string &xml_name,
    std::map<int, std::vector<int>> &mapping_table)
{
    m_config_oper->SetNodeDirectory(dir);
    std::deque<std::string> root_queue;
    root_queue.push_back(xml_name);
    if (!m_config_oper->InitConfigFileOperator(xml_name, root_queue))
    {
        return 1;
    }

    size_t child_size = m_config_oper->GetChildNum(root_queue);
    LOG_INFO("Child num = %d", child_size);
    std::string chile_name("");
    int root_id;
    std::vector<int> leafs;
    std::string leaf_str;
    for (size_t i = 0; i < child_size; ++i)
    {
        chile_name = UNDERLINE_FLAG + NumToString(i) + UNDERLINE_FLAG;
        root_queue.push_back(chile_name);
        m_config_oper->SetRootQueue(root_queue);
        if (!m_config_oper->GetParam("root_id", root_id, 1))
        {
            break;
        }
        if (!m_config_oper->GetParam("leaf_id", leaf_str, std::string("")))
        {
            break;
        }
        LOG_INFO("leaf string: %s", leaf_str.c_str());
        SplitStringToIntList(leaf_str, leafs);
        std::sort(leafs.begin(), leafs.end());
        mapping_table.insert(std::pair<int, std::vector<int>>(root_id, leafs));
        root_queue.pop_back();
    }
    LOG_INFO("All availabal mapping relationship in table has been read!");
    return 0;
}

int CFileInterface::ReadMappingTableFromXml(const std::string &dir, const std::string &xml_name,
    std::map<int, std::vector<NodeInsertToBaseStru> > &mapping_table)
{
    m_config_oper->SetNodeDirectory(dir);
    std::deque<std::string> root_queue;
    root_queue.push_back(xml_name);
    if (!m_config_oper->InitConfigFileOperator(xml_name, root_queue))
    {
        return 1;
    }

    size_t child_size = m_config_oper->GetChildNum(root_queue);
    LOG_INFO("Child num = %d", child_size);
    std::string chile_name(""), leaf_pre("leaf_");
    int root_id = 0, leaf_size = 0, tmpt = 0;
    std::vector<NodeInsertToBaseStru> leafs;
    NodeInsertToBaseStru leaf_item;
    for (size_t i = 0; i < child_size; ++i)
    {
        chile_name = UNDERLINE_FLAG + NumToString(i) + UNDERLINE_FLAG;
        root_queue.push_back(chile_name);
        m_config_oper->SetRootQueue(root_queue);
        if (!m_config_oper->GetParam("root_id", root_id, 1, false, false))
        {
            break;
        }

        leafs.clear();
        leaf_size = m_config_oper->GetSpecialNodeNumInOneLevel(root_queue, leaf_pre, true);
        for(int j = 0; j < leaf_size; ++j)
        {
            chile_name = leaf_pre + NumToString(j);
            root_queue.push_back(chile_name);
            m_config_oper->SetRootQueue(root_queue);
            root_queue.pop_back();
            if (!m_config_oper->GetParam("id", leaf_item.area_id, 0))
            {
                break;
            }
            if (!m_config_oper->GetParam("insert_type", tmpt, 0))
            {
                break;
            }
            leaf_item.insert_type = (NodeInsertTypeEnum)tmpt;
            LOG_INFO("insert type = %d", leaf_item.insert_type);
            //insert or replace node need read other properties
            if(NITYPE_KEEP == leaf_item.insert_type)
            {
                leafs.push_back(leaf_item);
                continue;
            }

            if (!m_config_oper->GetParam("insert_index", leaf_item.insert_index, 0))
            {
                break;
            }
            if (!m_config_oper->GetParam("motion_type", tmpt, 0))
            {
                break;
            }
            leaf_item.node_info.motion_type = (ServerMotionTypeEnum)tmpt;
            if (!m_config_oper->GetParam("turn_radius", leaf_item.node_info.turn_radius, 0.))
            {
                break;
            }
            if (!m_config_oper->GetParam("static_work_index", leaf_item.node_info.static_work_index, 0))
            {
                break;
            }
            if (!m_config_oper->GetParam("left_orientation", leaf_item.node_info.left_orientation, true))
            {
                break;
            }
            if (!m_config_oper->GetParam("target_vel", leaf_item.node_info.target_vel, 100.))
            {
                break;
            }
            leafs.push_back(leaf_item);
        }

//        std::sort(leafs.begin(), leafs.end());
        mapping_table.insert(std::pair<int, std::vector<NodeInsertToBaseStru>>(root_id, leafs));
        root_queue.pop_back();
    }
    LOG_INFO("All availabal mapping relationship in table has been read!");
    return 0;
}

int CFileInterface::ReadConnNodeInfoFromXml(const std::string &dir, const std::string &xml_name,
    std::map<int, std::map<int, PathPlanNodeInfoStru>> &conn_info)
{
    m_config_oper->SetNodeDirectory(dir);
    std::deque<std::string> root_queue;
    root_queue.push_back(xml_name);
    if (!m_config_oper->InitConfigFileOperator(xml_name, root_queue))
    {
        return 1;
    }

    size_t child_size = m_config_oper->GetChildNum(root_queue);
    LOG_INFO("Child num = %d", child_size);
    std::string chile_name(""), leaf_pre("leaf_");
    int root_id = 0, leaf_size = 0, tmpt = 0;
    std::map<int, PathPlanNodeInfoStru> leafs;
    PathPlanNodeInfoStru leaf_item;
    for (size_t i = 0; i < child_size; ++i)
    {
        chile_name = UNDERLINE_FLAG + NumToString(i) + UNDERLINE_FLAG;
        root_queue.push_back(chile_name);
        m_config_oper->SetRootQueue(root_queue);
        if (!m_config_oper->GetParam("root_id", root_id, 1))
        {
            break;
        }

        leafs.clear();
        leaf_size = m_config_oper->GetSpecialNodeNumInOneLevel(root_queue, leaf_pre, true);
        for(int j = 0; j < leaf_size; ++j)
        {
            chile_name = leaf_pre + NumToString(j);
            root_queue.push_back(chile_name);
            m_config_oper->SetRootQueue(root_queue);
            root_queue.pop_back();
            if (!m_config_oper->GetParam("id", leaf_item.id, 0))
            {
                break;
            }
            if (!m_config_oper->GetParam("motion_type", tmpt, 0))
            {
                break;
            }
            leaf_item.motion_type = (ServerMotionTypeEnum)tmpt;
            if (!m_config_oper->GetParam("vel_angle", leaf_item.vel_angle, 0.))
            {
                break;
            }
            leaf_item.vel_angle = SlamCommon::DegToRad(leaf_item.vel_angle);
            if (!m_config_oper->GetParam("turn_radius", leaf_item.turn_radius, 0.))
            {
                break;
            }
            if (!m_config_oper->GetParam("static_work_index", leaf_item.static_work_index, 0))
            {
                break;
            }
            if (!m_config_oper->GetParam("left_orientation", leaf_item.left_orientation, true))
            {
                break;
            }
            if (!m_config_oper->GetParam("target_vel", leaf_item.target_vel, 100.))
            {
                break;
            }
            leafs.insert(std::pair<int, PathPlanNodeInfoStru>(leaf_item.id, leaf_item));
        }

        conn_info.insert(std::pair<int, std::map<int, PathPlanNodeInfoStru>>(root_id, leafs));
        root_queue.pop_back();
    }
    LOG_INFO("All availabal connect node info has been read!");
    return 0;
}

int CFileInterface::SaveErrorList(const std::map<unsigned short, std::string> &error_info_map,
    const std::map<unsigned short, MainControl::ERROR_ATTRIBUTE_STRU> &error_attr_map,
    const std::string &file_name, const std::string &project_name)
{
    m_config_oper->InitConfigFileOperatorForWriting();

    boost::property_tree::ptree *root_tree = m_config_oper->GetRootTree();
    //write general part of reflector_map
    boost::property_tree::ptree root1, root2;

    root1.put<std::string>("project_name", project_name);
    std::map<unsigned short, std::string>::const_iterator info_iter = error_info_map.begin();
    std::map<unsigned short, MainControl::ERROR_ATTRIBUTE_STRU>::const_iterator attr_iter = error_attr_map.begin();
    int index = 0;
    for(; (info_iter != error_info_map.end() && attr_iter != error_attr_map.end()); ++info_iter, ++attr_iter)
    {
        root2.clear();
        root2.put<unsigned char>("code", info_iter->first);
        root2.put<std::string>("info", info_iter->second);
        root2.put<bool>("attr", attr_iter->second.m_ResetEnable);
        root1.add_child(UNDERLINE_FLAG + NumToString(index++) + UNDERLINE_FLAG, root2);
    }
    root_tree->add_child("error_list", root1);
    m_config_oper->WriteXmlFile(file_name);
    m_config_oper->UninitConfigFileOperator();
    return 0;
}

int CFileInterface::GetErrorList(std::map<unsigned short, std::string> &error_info_map,
    std::map<unsigned short, MainControl::ERROR_ATTRIBUTE_STRU> &error_attr_map,
    std::string &project_name, const std::string &file_name)
{
    std::string xml_path(file_name);
    if(xml_path.empty())
    {
        m_config_oper->SetNodeDirectory(DIR_CONFIGURATION_FILE_ID + std::string(DIR_FLAG) + CFG_MAIN_TASK_ID);
        if(!m_config_oper->InitConfigFileOperator(XML_ERROR_LIST_ID, XML_ERROR_LIST_ID))
        {
            return 1;
        }
    }
    else
    {
        int index = xml_path.find_last_of(DIR_FLAG);
        if(index < 0)
        {
            index = xml_path.find_last_of(DIR_INVERSE_FLAG);
        }
        std::string pre = xml_path.substr(0, index);
        std::string xml = xml_path.substr(index + 1, xml_path.length());
        xml = xml.substr(0, xml.find_last_of(SINGLE_DOT_FLAG));
        m_config_oper->SetNodeDirectory(pre);
        if(!m_config_oper->InitConfigFileOperator(xml, xml))
        {
            return 2;
        }
    }
    boost::property_tree::ptree *root_tree = m_config_oper->GetRootTree();
    //get error list information
    GetXMLRoot(error_info_map, error_attr_map, project_name, *root_tree);
    return 0;
}

void CFileInterface::GetXMLRoot(std::map<unsigned short, std::string> &error_info_map,
                                 std::map<unsigned short, MainControl::ERROR_ATTRIBUTE_STRU> &error_attr_map,
                                 std::string &project_name, boost::property_tree::ptree root_tree)
{
    bool get_node = false;
    unsigned short code = 0;
    std::string info("");
    MainControl::ERROR_ATTRIBUTE_STRU attr;
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v1, root_tree)
    {
        if (v1.second.empty())
        {
            if("<xmlcomment>" == v1.first)
            {
                continue;
            }
            if("project_name" == v1.first)
            {
                project_name = v1.second.data();
                continue;
            }
            get_node = true;
            if ("code" == v1.first)
            {
                code = SlamCommon::StringToNum<unsigned short>(v1.second.data());
            }
            else if ("info" == v1.first)
            {
                info = v1.second.data();
            }
            else if ("attr" == v1.first)
            {
                attr.m_ResetEnable = ("true" == v1.second.data()) ? true : false;
            }
        }
        else
        {
            GetXMLRoot(error_info_map, error_attr_map, project_name, v1.second);
        }
    }
    if (get_node)
    {
        error_info_map.insert(std::pair<unsigned short, std::string>(code, info));
        error_attr_map.insert(std::pair<unsigned short, MainControl::ERROR_ATTRIBUTE_STRU>(code, attr));
        get_node = false;
    }
}

/*********************************************************************
function: 将字符串写入txt文件
*********************************************************************/
void CFileInterface::WriteMultiLineStringToTxt(const std::string &dir_name,
    const std::string &txt_name, const std::vector<std::string> &strs)
{
    std::string dir(dir_name);
    AddDirectoryFlag(dir);
    const std::string complete_path = m_cur_path + dir + txt_name + TXT_ID;
    std::ofstream str_file(complete_path, std::ios::out | std::ios::binary);

    for(std::vector<std::string>::const_iterator it = strs.begin(); it != strs.end(); ++it)
    {
        m_config_oper->WriteStringToFile(str_file, *it + RETURN_FLAG);
    }
    str_file.close();
    // LOG_INFO("Write %s successfully!", complete_path.c_str());
}

/*********************************************************************
function: 将字符串写入txt文件
*********************************************************************/
bool CFileInterface::ReadStringFromTxt(const std::string &dir_name,
    const std::string &txt_name, std::vector<std::string> &strs)
{
    std::string str_tempt;
    std::string dir(dir_name);
    AddDirectoryFlag(dir);
    const std::string complete_path = m_cur_path + dir + txt_name + TXT_ID;
    std::ifstream infile;

    infile.open(complete_path.c_str());

    if (!infile.is_open())
    {
        LOG(WARNING) << "Open %s failed!" << complete_path.c_str();
        return false;
    }

    while (infile.good() && !infile.eof())
    {
        getline(infile, str_tempt, '\n');
        if (str_tempt.empty())
        {
            continue;
        }
        strs.push_back(str_tempt);
    }

    infile.close();
    LOG_INFO("Read %s successfully!", complete_path.c_str());
    return true;
}

int CFileInterface::ReadObsAreaFromXml(const std::string &dir,
                                       const std::string &xml_name,
                                       std::map<int, ObsGroupItemStru> &obs_areas)
{
    m_config_oper->SetNodeDirectory(dir);
    std::deque<std::string> root_queue;
    root_queue.push_back(xml_name);
    if (!m_config_oper->InitConfigFileOperator(xml_name, root_queue))
    {
        return 1;
    }

    size_t child_size = m_config_oper->GetChildNum(root_queue);
    LOG_INFO("Child num = %d", child_size);
    std::string chile_name(""), group_pre("group"), area_pre("area"), point_pre("point");
    int area_size = 0,leaf_size = 0;
    ObsGroupItemStru group_item;
    ObsAreaItemStru area_item;
    SlamCommon::Pose3D end_pt;
    int type = 0;
    //guoqiang
    int state;
    for (size_t i = 0; i < child_size; ++i)
    {
        chile_name = group_pre + NumToString(i);
        root_queue.push_back(chile_name);
        m_config_oper->SetRootQueue(root_queue);

        if (!m_config_oper->GetParam("active", group_item.active, false))
        {
            break;
        }
        if (!m_config_oper->GetParam("type", type, 0))
        {
            break;
        }
        if(type >= (int)AREATYPE_NUM)
        {
            return 2 + RETURN_FALSE_MAX_NUM * (int)AREATYPE_NUM;
        }
        group_item.type = (SlamCommon::ObsAreaTypeEnum)type;

        //read area info
        area_size = m_config_oper->GetSpecialNodeNumInOneLevel(
                    root_queue, area_pre, true);
        group_item.areas.clear();
        group_item.areas.reserve(area_size);
        for (int j = 0; j < area_size; ++j)
        {
            chile_name = area_pre + NumToString(j);
            root_queue.push_back(chile_name);
            leaf_size = m_config_oper->GetSpecialNodeNumInOneLevel(
                        root_queue, point_pre, true);
            area_item.end_pts.clear();
            area_item.end_pts.reserve(area_size);
            //把激光所在的(0, 0)点放入  ---guoqiang不再把(0, 0)点放入
            end_pt.Reset();
            //area_item.end_pts.push_back(end_pt);
            // get state

            m_config_oper->SetRootQueue(root_queue);
            if (m_config_oper->GetParam("state", state, 0))
            {
                area_item.state = state;
                LOG_INFO("state get! %d",state);
            }
            else{
                LOG_ERROR(" get state error!");
            }
            for(int k = 0; k < leaf_size; ++k)
            {
                chile_name = point_pre + NumToString(k);
                root_queue.push_back(chile_name);
                m_config_oper->SetRootQueue(root_queue);
                root_queue.pop_back();
                if (!m_config_oper->GetParam("x", end_pt.x, 0.))
                {
                    break;
                }
                if (!m_config_oper->GetParam("y", end_pt.y, 0.))
                {
                    break;
                }
                area_item.end_pts.push_back(end_pt);
            }
            root_queue.pop_back();
            if(area_item.end_pts.size() < 3)
            {
                return 3 + RETURN_FALSE_MAX_NUM * group_item.areas.size();
            }
            group_item.areas.push_back(area_item);
        }

        obs_areas.insert(std::pair<int, ObsGroupItemStru>(i, group_item));
        root_queue.pop_back();
    }
    LOG_INFO("All availabal connect node info has been read!");
    return 0;
}

} // namespace SlamCommon
