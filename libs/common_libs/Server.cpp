/* ============================================================
 *  Proyect  : I-Drone                                   
 *  Filename : Server.cpp                  
 *  Author   : Iván Gutiérrez                            
 *  License  : GNU General Public License v3.0           
 *
 *  © 2025 Iván Gutiérrez.
 * ============================================================
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "Server.h"


Server::Server(boost::asio::io_context& io_context, const tcp::endpoint& endpoint)
        : io_context_(io_context), acceptor_(io_context, endpoint) 
{
}

Server::Server(boost::asio::io_context& io_context, const tcp::endpoint& endpoint, const handlers &handlers)
        : io_context_(io_context), acceptor_(io_context, endpoint), handlers_(handlers)
{
}

Server::~Server()
{
    try {
        if (acceptor_.is_open()) {
            acceptor_.close();
        }

        if (current_client_ && current_client_->is_open()) {
            boost::system::error_code ec;
            current_client_->shutdown(tcp::socket::shutdown_both, ec);
            current_client_->close(ec);
        }
    } catch (const std::exception& e) {}
}

void Server::set_handlers(const handlers &handlers)
{
    handlers_ = handlers;
}

void Server::start_listening()
{
    auto new_socket = std::make_shared<tcp::socket>(io_context_);

    acceptor_.async_accept(*new_socket,
        [this, new_socket](const boost::system::error_code& error) {
            if (error) {
                if (handlers_.call_error) handlers_.call_error(error);
            } else {
                current_client_ = new_socket;
                start_read(new_socket);
            }

            start_listening();
        });
}

void Server::start_read(std::shared_ptr<tcp::socket> socket)
{
    auto length_buf = std::make_shared<std::array<char, 4>>();

    // Read first 4 bytes
    boost::asio::async_read(*socket, boost::asio::buffer(*length_buf),
        [this, socket, length_buf](const boost::system::error_code& ec, std::size_t /*bytes_transferred*/) {
            if (ec) {
                if (handlers_.call_error) handlers_.call_error(ec);
                return;
            }

            uint32_t msg_size = 0;
            std::memcpy(&msg_size, length_buf->data(), 4);
            msg_size = ntohl(msg_size);

            auto data_buf = std::make_shared<std::vector<char>>(msg_size);

            // Read the exact bytes
            boost::asio::async_read(*socket, boost::asio::buffer(*data_buf),
                [this, socket, data_buf](const boost::system::error_code& read_ec, std::size_t /*bytes_transferred*/) {
                    if (read_ec) {
                        if (handlers_.call_error) handlers_.call_error(read_ec);
                        return;
                    }

                    if (handlers_.call_message) {
                        handlers_.call_message(std::string(data_buf->data(), data_buf->size()));
                    }

                    // Read next message
                    start_read(socket);
                });
        });
}

void Server::connect(const tcp::endpoint& endpoint)
{
    current_client_ = std::make_shared<tcp::socket>(io_context_);

    current_client_->async_connect(endpoint,
        [this](const boost::system::error_code& ec) {
            if (ec) {
                if (handlers_.call_error) handlers_.call_error(ec);
            } else {
                start_read(current_client_);
            }
        });
}

void Server::deliver(const std::string &message)
{
    if (!current_client_ || !current_client_->is_open())
        return;

    uint32_t len = htonl(static_cast<uint32_t>(message.size()));
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(&len, sizeof(len)));
    buffers.push_back(boost::asio::buffer(message));

    boost::asio::async_write(*current_client_, buffers,
        [this](const boost::system::error_code& ec, std::size_t /*bytes_transferred*/) {
            if (ec && handlers_.call_error)
                handlers_.call_error(ec);
        });
}