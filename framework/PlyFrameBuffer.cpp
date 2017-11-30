#include <PlyFrameBuffer.hpp>
#include <fstream>
#include <sstream>
#include <cassert>
#include <iostream>
#include <experimental/filesystem>

PlyFrameBuffer::PlyFrameBuffer(const Meta& m)
    : file_paths_()
    , parsers_()
    , meta_(m)
{
    initFilenames();
}

PlyFrameBuffer::~PlyFrameBuffer()
{
    std::map<unsigned, PlyParser*>::iterator it;
    for(it = parsers_.begin(); it != parsers_.end(); ++it)
        delete it->second;
}

void PlyFrameBuffer::setMeta(const Meta& m)
{
    meta_ = m;
    std::map<unsigned, PlyParser*>::iterator it;
    for(it = parsers_.begin(); it != parsers_.end(); ++it)
        delete it->second;
    parsers_.clear();
    initFilenames();
}

const Meta& PlyFrameBuffer::getMeta() const
{
    return meta_;
}

unsigned PlyFrameBuffer::getFrameCount() const
{
    return file_paths_.size();
}

PlyParser* PlyFrameBuffer::getFrame(unsigned frame)
{
    if(frame >= file_paths_.size())
        return nullptr;

    std::map<unsigned, PlyParser*>::iterator it;
    it = parsers_.find(frame);
    if(it == parsers_.end()){
        parsers_.insert(std::pair<unsigned, PlyParser*>(frame, new PlyParser));
        it = parsers_.find(frame);
        it->second->parse(file_paths_[frame]);    
    }
    return it->second;
}

void PlyFrameBuffer::initFilenames()
{
    unsigned digets = meta_.format.size(); 
    for(unsigned i = meta_.start_frame; i <= meta_.end_frame; ++i)
    {
        std::string f_name = std::to_string(i);
        while(f_name.size() < digets) {
            f_name = "0" + f_name; 
        }
        f_name = meta_.directory + f_name + ".ply";
        file_paths_.push_back(f_name);
    }
}
