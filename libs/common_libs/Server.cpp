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


Server::Server(boost::asio::io_context& io_context)
        : io_context_(io_context), acceptor_(io_context), is_listening_(false)
{
}

Server::Server(boost::asio::io_context& io_context,  const handlers &handlers)
        : io_context_(io_context), acceptor_(io_context), handlers_(handlers), is_listening_(false)
{
}

Server::~Server()
{
    server_close();
}

void Server::server_close()
{
    try {
        boost::system::error_code ec;
        
        if (acceptor_.is_open()) {
            acceptor_.close(ec);
        }

        if (current_client_) {
            auto socket = current_client_;
            current_client_.reset();
            
            if (socket && socket->is_open()) {
                socket->shutdown(tcp::socket::shutdown_both, ec);
                socket->close(ec);
            }
        }
    } catch (const std::exception& e) {}
}


void Server::set_handlers(const handlers &handlers)
{
    handlers_ = handlers;
}

void Server::start_listening(const tcp::endpoint& endpoint)
{
    boost::system::error_code ec;

    if (!is_listening_) {
        if (!acceptor_.is_open()) {
            acceptor_.open(endpoint.protocol(), ec);
            if (ec) {
                if (handlers_.call_error) handlers_.call_error(ec, Type_Error::CONNECTING);
                return;
            }
        }

        acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);

        acceptor_.bind(endpoint, ec);
        if (ec) {
            if (handlers_.call_error) handlers_.call_error(ec, Type_Error::CONNECTING);
            return;
        }

        acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
        if (ec) {
            if (handlers_.call_error) handlers_.call_error(ec, Type_Error::CONNECTING);
            return;
        }
        
        is_listening_ = true;
    }

    auto new_socket = std::make_shared<tcp::socket>(io_context_);

    acceptor_.async_accept(*new_socket,
        [this, new_socket](const boost::system::error_code& error) {
            if (error) {
                if (handlers_.call_error)
                    handlers_.call_error(error, Type_Error::CONNECTING);
            } else {
                if (handlers_.call_connect)
                    handlers_.call_connect();

                current_client_ = new_socket;
                start_read(current_client_);
            }
        });
}

void Server::accept_new_connection()
{
    if (!is_listening_) {
        return;
    }

    auto new_socket = std::make_shared<tcp::socket>(io_context_);

    acceptor_.async_accept(*new_socket,
        [this, new_socket](const boost::system::error_code& error) {
            if (error) {
                if (handlers_.call_error)
                    handlers_.call_error(error, Type_Error::CONNECTING);
            } else {
                if (handlers_.call_connect)
                    handlers_.call_connect();

                current_client_ = new_socket;
                start_read(current_client_);
            }
        });
}


void Server::start_read(std::shared_ptr<tcp::socket> socket)
{
    auto length_buf = std::make_shared<std::array<char, 4>>();

    // Read 4 first bytes
    boost::asio::async_read(*socket, boost::asio::buffer(*length_buf),
        [this, socket, length_buf](const boost::system::error_code& ec, std::size_t) {
            if (ec) {
                if (handlers_.call_error) handlers_.call_error(ec, Type_Error::READING);
                return;
            }

            uint32_t msg_size = 0;
            std::memcpy(&msg_size, length_buf->data(), 4);
            msg_size = ntohl(msg_size);

            auto data_buf = std::make_shared<std::vector<char>>(msg_size);

            // Read the exact length of the message
            boost::asio::async_read(*socket, boost::asio::buffer(*data_buf),
                [this, socket, data_buf](const boost::system::error_code& read_ec, std::size_t) {
                    if (read_ec) {
                        if (handlers_.call_error) handlers_.call_error(read_ec, Type_Error::READING);
                        return;
                    }

                    if (handlers_.call_message) {
                        handlers_.call_message(
                            std::string(data_buf->data(), data_buf->size())
                        );
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
                if (handlers_.call_error) handlers_.call_error(ec, Type_Error::CONNECTING);
            } else {
                if (handlers_.call_connect) handlers_.call_connect();
                start_read(current_client_);
            }
        });
}

void Server::deliver(const std::string &message)
{
    auto client = current_client_;
    if (!client || !client->is_open())
        return;

    uint32_t len = htonl(static_cast<uint32_t>(message.size()));
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(&len, sizeof(len)));
    buffers.push_back(boost::asio::buffer(message));

    boost::asio::async_write(*client, buffers,
        [this, client](const boost::system::error_code& ec, std::size_t /*bytes_transferred*/) {
            if (ec && handlers_.call_error)
                handlers_.call_error(ec, Type_Error::SENDING);
        });
}