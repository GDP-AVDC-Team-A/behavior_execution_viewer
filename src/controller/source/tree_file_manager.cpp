/*TreeFileManager
  @author  Jorge Luis Pascual, Carlos Valencia.
  @date    07-2017
  @version 2.0
*/
#include "../include/tree_file_manager.h"

TreeFileManager::TreeFileManager()
{
         std::cout << "file TreeFileManager, function constructor"<< std::endl;

    n.param<std::string>("robot_namespace", drone_id_namespace, "drone1");
    n.param<std::string>("catalog_path", behavior_catalog_path, "${AEROSTACK_PROJECT}/configs/mission/behavior_catalog.yaml");
    //std::cout << "/"<<drone_id_namespace<<"/"<<consult_available_behaviors<< std::endl;
    sleep(3);
    bool loaded = loadConfiguration(behavior_catalog_path);
    available_behaviors= getBehaviors();

}

TreeFileManager::~TreeFileManager()
{

}

void TreeFileManager::saveTree(TreeItem* root_item, std::string save_route)
{
           std::cout << "file TreeFileManager, function saveTree"<< std::endl;

    id_count = 1;
    int parent = id_count;
    std::string aux;
    YAML::Node node;
    std::ofstream fout(save_route);
    emitter << YAML::BeginMap; //Mapa Global
    emitter << YAML::Key << "nodes"; 
    if(root_item != nullptr)
    {   
      emitter << YAML::BeginSeq; //Secuencia de nodos
      emitter << YAML::BeginMap; //Mapa dentro de cada nodo
      emitter << YAML::Key << "id" << YAML::Value << id_count;
      emitter << YAML::Key << "node_name" << YAML::Value << root_item->getNodeName();
      emitter << YAML::Key << "node_type" << YAML::Value << root_item->nodeTypeToString(root_item->getNodeType());
      emitter << YAML::Key << "task" << YAML::Value << root_item->getBehaviorType();
      emitter << YAML::Key << "parameters" << YAML::Value << root_item->getNodeAttributes();
      emitter << YAML::Key << "is_recurrent" << YAML::Value << root_item->isRecurrent();
      emitter << YAML::Key << "activate" << YAML::Value << root_item->isActivated();
      emitter << YAML::Key << "parent" << YAML::Value << 0;
      emitter << YAML::EndMap;
      if(root_item->childCount()>0)
      {
        for(int i = 0; i < root_item->childCount(); i++)
        {
          id_count++;
          recursive_save(root_item->child(i), id_count, parent);
        }
      }
      //After this loop we have all the nodes set up in the emitter
      emitter << YAML::EndSeq;
      //TODO: save variables
    }
    else 
    {
      emitter << YAML::Value << YAML::Null;
    }
    //aux = " " + variables + " ";
    //emitter << YAML::Key << "variables" << YAML::Value << aux ;
    emitter << YAML::EndMap;
    fout << emitter.c_str();
}

void TreeFileManager::recursive_save(TreeItem* item, int id, int parent)
{ 
             std::cout << "file TreeFileManager, function recursive_save"<< std::endl;

    emitter << YAML::BeginMap; //Mapa dentro de cada nodo
    emitter << YAML::Key << "id" << YAML::Value << id;
    emitter << YAML::Key << "node_name" << YAML::Value << item->getNodeName();
    emitter << YAML::Key << "node_type" << YAML::Value << item->nodeTypeToString(item->getNodeType());
    emitter << YAML::Key << "task" << YAML::Value << item->getBehaviorType();
    emitter << YAML::Key << "parameters" << YAML::Value << item->getNodeAttributes();
    emitter << YAML::Key << "is_recurrent" << YAML::Value << item->isRecurrent();
    emitter << YAML::Key << "activate" << YAML::Value << item->isActivated();
    emitter << YAML::Key << "parent" << YAML::Value << parent;
    emitter << YAML::EndMap;
    if(item->childCount()>0)
    {
      int parent_aux = id_count;
      for(int i = 0; i < item->childCount(); i++)
      {
        id_count++;
        recursive_save(item->child(i), id_count, parent_aux);
      }
    }
}

TreeItem* TreeFileManager::loadTree(std::string load_route)
{
  std::cout << "file TreeFileManager, function loadTree"<< std::endl;

  TreeItem* root = nullptr;
  QMessageBox error_message;

  YAML::Node archive = YAML::LoadFile(load_route);
  YAML::Node nodes;
  //YAML::Node variables;

  if (!(nodes = archive["nodes"]))
  {
    error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
    error_message.setText(QString::fromStdString("The mission file is badly structured. Unable to import."));
    error_message.exec();
    return nullptr;
  }

  std::vector<TreeItem*> item;
  std::map<int, TreeItem*> m;
  std::map<int, int> map_parent;



  std::vector <std::string> node_types = {"TASK", "REPEAT_UNTIL_FAIL", "PARALLEL", "REPEATER", "QUERY", "ADD_BELIEF", "REMOVE_BELIEF", "SUCCEEDER", "INVERTER", "SELECTOR", "SEQUENCE", "SELECTOR"};

  int parent;
  if(archive.size() != 0) 
  { 
    for (std::size_t i=0;i<nodes.size();i++) {
      YAML::Node node = nodes[i];
      try{
          parent = node["parent"].as<int>();
      }
      catch (const std::exception &e)
              {
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect parent format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
              }

      /*for (auto const& i: available_behaviors) {
                               std::cout << i << " ";
                       }*/
                     
      if(parent == 0)
      {
        TreeItem * item = new TreeItem(0);
        int id;
        try{
        id = node["id"].as<int>();
        }
        catch (const std::exception &e)
        {

          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect node id format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        std::string node_name;
        bool activate;
        bool recurrent;
        std::string parameters;
        NodeType type;
        std::string typeaux;
        std::string btype;
        try
        {
           node_name = node["node_name"].as<std::string>(); 
        } catch (const std::exception &e)
        {
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect node name format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        try
        {
           type = item->stringToNodeType(node["node_type"].as<std::string>()); 
           typeaux = node["node_type"].as<std::string>();
           btype = node["task"].as<std::string>(); 
        } catch (const std::exception &e)
        {
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect behavior type format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        try
        {
           if (std::find(available_behaviors.begin(), available_behaviors.end(), btype) == available_behaviors.end())
           {                
            for (int i = 0; i < available_behaviors.size(); i++) {
            std::cout << available_behaviors.at(i) << ' ';
        } 
               std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect behavior_type \"" << btype << "\"\n";
               //std::cout << "macarron"<< std::endl;
               for (auto const& i: available_behaviors) {
                               std::cout << i << " ";
                       }
                     
               error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
               error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
               error_message.exec();
               return nullptr;
           }
           recurrent = node["is_recurrent"].as<bool>(); 
        } catch (const std::exception &e)
        {
                          

          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect recurrent format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        try
        {
           activate = node["activate"].as<bool>();
        } catch (const std::exception &e)
        {
                          

          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect activate format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        try
        {
           parameters = node["parameters"].as<std::string>(); 
           if (std::find(node_types.begin(), node_types.end(), typeaux) == node_types.end())
           {
                            
               std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect node type \"" << typeaux << "\"\n";
               error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
               error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
               error_message.exec();
               return nullptr;
           }


           
           bool resi=checkParameters(btype,parameters);
           if (!resi){
        throw std::invalid_argument( "received invalid value" );
           }
           

        } catch (const std::exception &e)
        {
                          

          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect parameters format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of parameters for behavior in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        item->modifyNode(node_name, type, btype, recurrent, activate, parameters);
        m[id] = item;
        map_parent[id] = parent;
      } else 
      {
        TreeItem * item = new TreeItem(m[parent]);
        YAML::Node node = nodes[i];
        int id;
        std::string node_name;
        NodeType type;
        std::string typeaux;
        std::string btype;
        bool recurrent;
        bool activate;
        std::string parameters;

        try
        {
           id = node["id"].as<int>(); 
        }
        catch (const std::exception &e)
        {
                          

          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect node id format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        try
        {
           node_name = node["node_name"].as<std::string>();
        } catch (const std::exception &e)
        {
                          

          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect node name format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        try
        {
           typeaux = node["node_type"].as<std::string>();
           btype = node["task"].as<std::string>();
           type = item->stringToNodeType(node["node_type"].as<std::string>()); 
        } catch (const std::exception &e)
        {
                          

          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect behavior type format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        try
        {
            if (std::find(available_behaviors.begin(), available_behaviors.end(), btype) == available_behaviors.end())
            {              

                std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect behavior_type \"" << btype << "\"\n";
                //std::cout << "sardina"<< std::endl;
                error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
                error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
                error_message.exec();
                return nullptr;
            }
           recurrent = node["is_recurrent"].as<bool>(); 
        } catch (const std::exception &e)
        {
                          

          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect recurrent format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        try
        {
           activate = node["activate"].as<bool>(); 
        } catch (const std::exception &e)
        {
                          
          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect activate format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        try
        {
           parameters = node["parameters"].as<std::string>(); 

           if (std::find(node_types.begin(), node_types.end(), typeaux) == node_types.end())
           {
                          

               std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect node type \"" << typeaux << "\"\n";
               error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
               error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of behavior tree in file "+ load_route + "."));
               error_message.exec();
               return nullptr;
           }


           //behavior_msg.name = btype;
           //behavior_msg.parameters = parameters;
           //check_format_msg_req.behavior = behavior_msg;
                      //std::cout <<"argumentos: "<<parameters<<std::endl;
           bool resi=checkParameters(btype,parameters);
           if (!resi){
        throw std::invalid_argument( "received invalid value" );

           }
           

        } catch (const std::exception &e)
        {
                          

          std::cout << "Wrong format of behavior tree in file \"" << load_route << "\"\nNode id " << std::to_string(i+1) << "\n" << "Incorrect parameters format" << "\n";
          error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
          error_message.setText(QString::fromStdString("Unable to import mission plan.\nWrong format of parameters for behavior in file "+ load_route + "."));
          error_message.exec();
          return nullptr;
        }
        item->modifyNode(node_name, type, btype, recurrent, activate, parameters);
        m[id] = item;
        map_parent[id] = parent;
      }


    }
    for (const auto& kv : map_parent) 
    {
      if (kv.second == 0) 
      {
        m[kv.first]->setRoot(true);
        root = m[kv.first];
      }
      else 
      {
        m[kv.second]->addChild(m[kv.first]);
      }
    }
    //std::string str;
       
    return root;
  } else 
  {
    error_message.setWindowTitle(QString::fromStdString("Loading mission error"));
    error_message.setText(QString::fromStdString("The mission file is empty. Unable to import."));
    error_message.exec();
    return nullptr;
  }
}



  /*-------------------------------------------------------*/
  /*BEHAVIOR_CATALOG*/
  /*-------------------------------------------------------*/
std::vector<std::string> TreeFileManager::getBehaviors()
{
    std::cout << "file TreeFileManager, function getBehaviors"<< std::endl;

 
  return behaviors_loaded;
}

bool TreeFileManager::loadConfiguration(std::string file_path)
{ 
      std::cout << "file TreeFileManager, function loadConfiguration"<< std::endl;

  std::cout<<"Catalog: "<<file_path<<std::endl;

try{
    YAML::Node yaml_node;
  yaml_node = YAML::LoadFile(file_path);
  if (yaml_node["tasks"]){
    for (YAML::const_iterator tasksi=yaml_node["tasks"].begin();
        tasksi!=yaml_node["tasks"].end();++tasksi) {
      std::map<std::string, std::vector<std::string>> parameters;
      std::string name=(*tasksi)["task"].as<std::string>();
      if((*tasksi)["task"]){
        behaviors_loaded.push_back((*tasksi)["task"].as<std::string>());

      }
      if((*tasksi)["parameters"]){
        for (YAML::const_iterator parametersIterator=(*tasksi)["parameters"].begin();
            parametersIterator!=(*tasksi)["parameters"].end();++parametersIterator){
            std::string nameParameters;
            std::vector<std::string> allowed;
            if((*parametersIterator)["parameter"]){
                nameParameters = (*parametersIterator)["parameter"].as<std::string>();

              }
            if((*parametersIterator)["allowed_values"]){
              try{
                 allowed = (*parametersIterator)["allowed_values"].as<std::vector<std::string>>();
              }
              catch(const std::exception& e){

                 std::cout<<"That parameter doesn't follow the Yaml rule"<< std::endl;
              }

             }

             parameters.insert(std::pair<std::string,std::vector<std::string>> (nameParameters,allowed));


            }
            behaviors_loaded_complete.insert(std::pair<std::string,std::map<std::string,std::vector<std::string>>>(name, parameters));

      }
      if (!(*tasksi)["parameters"])
      {
        behaviors_loaded_complete.insert(std::pair<std::string,std::map<std::string,std::vector<std::string>>>(name, parameters));

      }

    }
    return true;
  }
}
catch (const std::exception& e) { // referencia a base de un objeto polimorfo
     std::cout << e.what();
     return false;
}
    
}
bool TreeFileManager::checkParameters(std::string task,std::string parameters){
        std::cout << "file TreeFileManager, function checkParameters"<< std::endl;

   std::cout<< parameters<<std::endl;
    std::map<std::string,std::map<std::string, std::vector<std::string>>>::iterator it;

    it = behaviors_loaded_complete.find(task);
      if (it == behaviors_loaded_complete.end()){
        std::cout << "The task doesn't exist"<<std::endl;
        return false;
      }
      std::map<std::string,std::vector<std::string>> valid_parameters =behaviors_loaded_complete.at(task);
      if (valid_parameters.empty()){
          std::cout<< "The task doesn't have parameters"<<std::endl;
          return true;
      }
      YAML::Node node= YAML::Load(parameters);

      for (auto tag : node){

         auto parameter_name = tag.first.as<std::string>();
         if (parameter_name=="direction"){
            auto parameter_value= tag.second.as<std::string>();
            if (parameter_value=="FORWARD"||parameter_value=="BACKWARD" || parameter_value=="LEFT" || 
              parameter_value == "RIGHT")
              {return true;}
            else{
              return false;
            }

         }

         
         std::map<std::string,std::vector<std::string>>::iterator it2=valid_parameters.find(parameter_name);
         if (it2 == valid_parameters.end()){
            std::cout<< "That parameters are not in the catalog "<<std::endl;
            return false;
         }

         std::vector<std::string> value= valid_parameters.at(parameter_name);//valores correctos
                  
         if (value.empty()){
          return true;
         }
         auto  parameter_value= tag.second.as<int>();

         std::vector<int>numbers;
         std::string aux = ""; 
         
         if (value.size()>0){
           for (int j =0; j<value.size();j++){
                   numbers.push_back(stoi(value.at(j)));

             }
         int params = node[parameter_name].as<int>();
         if ((params<numbers[0] || params>numbers[1]))
          { std::cout<< "The parameters are not written correctly" << std::endl;
            return false;}
         }
         else{
          return true;
         }
        
 
      }

      std::cout<< "The parameters "<<parameters <<" of "<<task<<" are written correctly"<<std::endl;
      return true;


  }

